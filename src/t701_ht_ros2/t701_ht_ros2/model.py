"""SDK 2.0 motor-coordinate model and mesh self-collision checks.

This module never imports a motor driver. Pinocchio is loaded only when a model
is constructed. Collision checking samples paths; it is not a continuous
collision or physical stopping-distance guarantee.
"""
from pathlib import Path
import threading
import numpy as np
import yaml


def _vector(value, count, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (count,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label}: expected {count} finite values")
    return result.copy()


class SelfCollisionGuard:
    """当前 URDF 网格的离线自碰撞门禁，不创建 SDK 对象或打开通信设备。

    只排除同一刚体及关节直接连接的刚体；其余同臂、臂与本体以及
    已提供两侧配置时的双臂交叉对均检查。单侧配置不假设另一臂姿态。
    默认预留 1 mm；当前模型零位 J5/J7 的正常间隙约 2.25 mm。
    关节段按最大 0.025 rad 取样，不是连续碰撞检测或实测制动保证。
    URDF 没有的夹指、线缆和外部障碍物不在几何检查范围内。
    """

    def __init__(self, config_paths, margin=.001):
        import pinocchio as pin
        self.pin = pin
        from pathlib import Path
        import tempfile
        import xml.etree.ElementTree as ET
        from urllib.parse import unquote

        if not isinstance(config_paths, dict) or not config_paths:
            raise ValueError("碰撞检查必须提供非空的侧别与 YAML 路径映射")
        self.margin = float(margin)
        if not np.isfinite(self.margin) or self.margin < 0.:
            raise ValueError("碰撞预留距离必须为有限非负数")
        self.config_paths = {side: str(Path(path).resolve(strict=True))
                             for side, path in config_paths.items()}
        self.sides = tuple(self.config_paths)
        self.configs, urdf_paths = {}, set()
        for side, path in self.config_paths.items():
            with open(path, encoding="utf-8") as stream:
                config = yaml.safe_load(stream)
            self.configs[side] = config
            urdf_paths.add((Path(path).parent / config["urdf"]["file_path"]).resolve(strict=True))
        if len(urdf_paths) != 1:
            raise ValueError("双臂碰撞检查必须使用同一个完整 URDF 模型")
        self.urdf_path = str(next(iter(urdf_paths)))
        tree = ET.parse(self.urdf_path)
        xml_root = tree.getroot()
        urdf_dir = Path(self.urdf_path).parent
        # URDF 导出的相对 STL 路径独立于运行目录解析，不切换进程 cwd。
        for mesh in xml_root.findall(".//mesh"):
            source = mesh.get("filename", "")
            if source.startswith("file://"):
                candidate = Path(unquote(source[7:]))
            elif source.startswith("package://"):
                relative = unquote(source[10:])
                package, _, tail = relative.partition("/")
                candidates = [base / relative for base in (urdf_dir, *urdf_dir.parents)]
                candidates += [base / tail for base in (urdf_dir, *urdf_dir.parents)
                               if base.name == package]
                candidate = next((item for item in candidates if item.is_file()), None)
                if candidate is None:
                    raise FileNotFoundError(f"碰撞网格未找到: {source}")
            elif "://" in source:
                raise ValueError(f"碰撞网格不支持网络路径: {source}")
            else:
                candidate = urdf_dir / source
            candidate = candidate.resolve(strict=True)
            if not candidate.is_file():
                raise FileNotFoundError(f"碰撞网格不是文件: {source}")
            mesh.set("filename", str(candidate))
        self.model = self.pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", encoding="utf-8") as stream:
            tree.write(stream, encoding="unicode")
            stream.flush()
            self.geometry_model = self.pin.buildGeomFromUrdf(
                self.model, stream.name, self.pin.GeometryType.COLLISION,
            )
        if not self.geometry_model.ngeoms:
            raise ValueError("URDF 未提供碰撞几何，拒绝启用不完整的防碰撞检查")

        self.q_indices, self.joint_signs = {}, {}
        controlled, used_indices = {}, set()
        for side, config in self.configs.items():
            names = list(config["kinematics"]["joint_names"])
            signs = np.asarray(config["kinematics"]["joint_signs"], dtype=float)
            if len(names) != 7 or len(set(names)) != 7 or signs.shape != (7,) or not np.all(np.isin(signs, (-1., 1.))):
                raise ValueError(f"{side} 碰撞模型需要七个不重复关节及七个 ±1 坐标符号")
            if not all(self.model.existJointName(name) for name in names):
                raise ValueError(f"{side} 关节名称与碰撞模型不一致")
            joints = [self.model.joints[self.model.getJointId(name)] for name in names]
            if any(joint.nq != 1 or joint.nv != 1 for joint in joints):
                raise ValueError("碰撞检查仅支持当前七轴单自由度机械臂模型")
            indices = np.asarray([joint.idx_q for joint in joints], dtype=int)
            if used_indices.intersection(indices.tolist()):
                raise ValueError("碰撞模型两侧配置重复控制同一个关节")
            used_indices.update(indices.tolist())
            self.q_indices[side], self.joint_signs[side] = indices, signs
            controlled.update({name: side for name in names})

        links = {element.get("name") for element in xml_root.findall("link")}
        rigid_parent = {link: link for link in links}

        def rigid(link):
            while rigid_parent[link] != link:
                rigid_parent[link] = rigid_parent[rigid_parent[link]]
                link = rigid_parent[link]
            return link

        joints = []
        for element in xml_root.findall("joint"):
            parent = element.find("parent").get("link")
            child = element.find("child").get("link")
            joints.append((element.get("name"), element.get("type"), parent, child))
            if element.get("type") == "fixed":
                rigid_parent[rigid(child)] = rigid(parent)
        adjacent = {frozenset((rigid(parent), rigid(child)))
                    for _, kind, parent, child in joints if kind != "fixed"}
        # 标记每个连杆属于已追踪机械臂、静态本体或未追踪的运动分支。
        incoming = {child: (name, kind, parent) for name, kind, parent, child in joints}
        inactive = object()
        ownership = {}

        def owner(link):
            if link in ownership:
                return ownership[link]
            if link not in incoming:
                result = None
            else:
                name, kind, parent = incoming[link]
                result = controlled.get(name)
                if result is None:
                    result = owner(parent)
                    if kind != "fixed" and result is None:
                        result = inactive
            ownership[link] = result
            return result

        geometry_links = []
        geometry_owners = []
        for geometry in self.geometry_model.geometryObjects:
            link = self.model.frames[geometry.parentFrame].name
            if link not in links:
                raise ValueError(f"碰撞几何没有有效的 URDF 连杆: {geometry.name}")
            geometry_links.append(link)
            geometry_owners.append(owner(link))
            geometry.geometry.computeLocalAABB()
        represented = set(geometry_links)
        required = {link for link in links if owner(link) is not inactive}
        if not required.issubset(represented):
            missing = sorted(required - represented)
            raise ValueError(f"受检连杆缺少 collision 几何: {missing}")
        self.geometry_model.addAllCollisionPairs()
        self.active_pair_indices = []
        self.pair_links = {}
        for index, pair in enumerate(self.geometry_model.collisionPairs):
            a, b = pair.first, pair.second
            side_a, side_b = geometry_owners[a], geometry_owners[b]
            if side_a is inactive or side_b is inactive or side_a is None and side_b is None:
                continue
            link_a, link_b = geometry_links[a], geometry_links[b]
            rigid_a, rigid_b = rigid(link_a), rigid(link_b)
            if rigid_a == rigid_b or frozenset((rigid_a, rigid_b)) in adjacent:
                continue
            self.active_pair_indices.append(index)
            self.pair_links[index] = (link_a, link_b)
        if not self.active_pair_indices:
            raise ValueError("碰撞检查没有有效的非相邻连杆检测对")
        self.geometry_data = self.pin.GeometryData(self.geometry_model)
        for index in self.active_pair_indices:
            request = self.geometry_data.collisionRequests[index]
            if not hasattr(request, "security_margin"):
                raise RuntimeError("当前碰撞库不支持预留间距，请安装项目要求的 hpp-fcl")
            request.security_margin = self.margin
            request.enable_contact = False
        self.scope = ("双臂互撞、各臂自碰撞及碰本体" if len(self.sides) == 2
                      else "所选臂自碰撞及碰本体；未提供另一臂实时姿态，不检查双臂互撞")
        print(f"[自碰撞检查] {self.scope}；{len(self.active_pair_indices)} 对网格，"
              f"预留 {self.margin * 1000.:g} mm", flush=True)

    def _positions(self, positions):
        if not isinstance(positions, dict) or set(positions) != set(self.sides):
            raise ValueError("碰撞检查必须同时提供所有受检机械臂的位置")
        return {side: _vector(positions[side], 7, f"{side}碰撞检查位置")
                for side in self.sides}

    def _first_collision(self, positions):
        full = self.pin.neutral(self.model)
        for side in self.sides:
            full[self.q_indices[side]] = self.joint_signs[side] * positions[side]
        self.pin.updateGeometryPlacements(
            self.model, self.data, self.geometry_model, self.geometry_data, full,
        )
        for index in self.active_pair_indices:
            # BVH 查询以 security_margin 判定；只有靠近时才算精确距离，
            # 避免高精度导出网格在每个远离的检测对上求最近点。
            if self.pin.computeCollision(self.geometry_model, self.geometry_data, index):
                distance = float(self.pin.computeDistance(
                    self.geometry_model, self.geometry_data, index,
                ).min_distance)
                if not np.isfinite(distance):
                    raise RuntimeError("碰撞网格距离计算无效")
                if distance <= self.margin:
                    return {"pair": self.pair_links[index], "distance": distance,
                            "sample": 0.}
        return None


    def first_collision(self, positions):
        """返回第一个碰撞/间距不足的连杆对，安全时返回 None。"""
        return self._first_collision(self._positions(positions))

    def validate_path(self, start, target, *, max_step=.025):
        """检查起点、终点和段内采样；同时插值所有受检机械臂。"""
        start, target = self._positions(start), self._positions(target)
        max_step = float(max_step)
        if not np.isfinite(max_step) or not 0. < max_step <= .025:
            raise ValueError("碰撞路径采样步长必须在 0～0.025 rad 之间")
        displacement = max(float(np.max(abs(target[side] - start[side])))
                           for side in self.sides)
        segments = max(1, int(np.ceil(displacement / max_step)))
        if segments > 256:
            raise ValueError("单次碰撞检查的关节跨度过大，请分段生成运动指令")
        for sample in range(segments + 1):
            blend = sample / segments
            positions = {side: start[side] + blend * (target[side] - start[side])
                         for side in self.sides}
            result = self._first_collision(positions)
            if result is not None:
                result["sample"] = blend
                return result
        return None

class ArmModel:
    """ROS commands use SDK motor coordinates; published state uses URDF axes.

    Names accepted by command interfaces are ``left_joint1`` ...
    ``left_joint7`` and their right-arm equivalents. The URDF names are
    retained verbatim for robot_state_publisher. No model offsets are applied
    to recorded trajectories or to hardware state. New zero means all motor
    coordinates zero; the raised VR posture adds +/-pi/2 at J7.
    """

    def __init__(self, description_dir, sides=("left", "right"), collision_margin=.001):
        self.description_dir = Path(description_dir).resolve(strict=True)
        self.sides = tuple(sides)
        if (not self.sides or len(self.sides) != len(set(self.sides))
                or not set(self.sides) <= {"left", "right"}):
            raise ValueError("sides must select left, right or both, without duplicates")
        self.config_paths = {
            side: self.description_dir / "config" / f"T701-HT_{side}.yaml"
            for side in self.sides
        }
        self.guard = SelfCollisionGuard(self.config_paths, margin=collision_margin)
        self._collision_lock = threading.RLock()
        self.configs = self.guard.configs
        self.pin = self.guard.pin
        self.model = self.guard.model
        self.data = self.model.createData()
        self.urdf_path = Path(self.guard.urdf_path)
        self.names = tuple(f"{side}_joint{joint}" for side in self.sides for joint in range(1, 8))
        self.joint_names = self.names
        self.count = len(self.names)
        self.dof = self.count
        self.urdf_names = tuple(name for side in self.sides
                                for name in self.configs[side]["kinematics"]["joint_names"])
        self.signs = np.concatenate([self.guard.joint_signs[side] for side in self.sides])
        self.q_indices = np.concatenate([self.guard.q_indices[side] for side in self.sides])
        self.v_indices = np.array([
            self.model.joints[self.model.getJointId(name)].idx_v for name in self.urdf_names
        ], dtype=int)
        self.end_frames = {}
        for side in self.sides:
            name = self.configs[side]["urdf"]["end_effector_link"]
            if not self.model.existFrame(name, self.pin.BODY):
                raise ValueError(f"Unknown end effector frame: {name}")
            self.end_frames[side] = self.model.getFrameId(name, self.pin.BODY)
        for attribute, field in (("velocity", "velocity_limits"),
                                  ("acceleration", "acceleration_limits"),
                                  ("torque", "max_torque")):
            values = np.concatenate([
                _vector(self.configs[side]["robot"][field], 7, f"{side}.{field}")
                for side in self.sides
            ])
            if np.any(values <= 0.):
                raise ValueError(f"{field} must be positive")
            setattr(self, attribute, values)
        self.lower = np.concatenate([
            _vector(self.configs[s]["robot"]["joint_limits"]["lower"], 7, f"{s}.lower")
            for s in self.sides
        ])
        self.upper = np.concatenate([
            _vector(self.configs[s]["robot"]["joint_limits"]["upper"], 7, f"{s}.upper")
            for s in self.sides
        ])
        if np.any(self.lower >= self.upper) or np.any(self.lower > 0.) or np.any(self.upper < 0.):
            raise ValueError("Invalid joint limits or motor zero excluded")
        a, b = self.lower * self.signs, self.upper * self.signs
        if (np.any(np.minimum(a, b) < self.model.lowerPositionLimit[self.q_indices] - 1e-4)
                or np.any(np.maximum(a, b) > self.model.upperPositionLimit[self.q_indices] + 1e-4)):
            raise ValueError("YAML motor limits exceed signed URDF joint limits")
        self.model.gravity.linear = np.array([0., 0., -9.81])

    def zeros(self):
        return np.zeros(self.count)

    def vr_pose(self):
        left = np.array([.3, 0., 0., -1.87, 0., 0., np.pi / 2.])
        return np.concatenate([left if s == "left" else -left for s in self.sides])

    def validate(self, q):
        q = _vector(q, self.count, "joint positions")
        if np.any(q < self.lower - 1e-9) or np.any(q > self.upper + 1e-9):
            index = int(np.argmax(np.maximum(self.lower - q, q - self.upper)))
            raise ValueError(f"{self.names[index]} outside configured limits")
        return q

    def split(self, q):
        q = _vector(q, self.count, "joint vector")
        return {side: q[index * 7:(index + 1) * 7].copy()
                for index, side in enumerate(self.sides)}

    def motor_to_model(self, q):
        q = _vector(q, self.count, "joint positions")
        full = self.pin.neutral(self.model)
        full[self.q_indices] = self.signs * q
        return full

    def forward(self, q):
        full = self.motor_to_model(q)
        self.pin.forwardKinematics(self.model, self.data, full)
        self.pin.updateFramePlacements(self.model, self.data)
        return {side: np.array(self.data.oMf[frame].homogeneous, copy=True)
                for side, frame in self.end_frames.items()}

    def gravity(self, q):
        full = self.motor_to_model(q)
        result = self.pin.computeGeneralizedGravity(self.model, self.data, full)
        return np.clip(self.signs * result[self.v_indices], -self.torque, self.torque)

    def joint_states(self, q, v=None):
        q = _vector(q, self.count, "joint positions")
        v = np.zeros(self.count) if v is None else _vector(v, self.count, "joint velocities")
        return self.urdf_names, self.signs * q, self.signs * v

    def collision(self, start, target):
        start, target = self.validate(start), self.validate(target)
        with self._collision_lock:
            return self.guard.validate_path(self.split(start), self.split(target))

