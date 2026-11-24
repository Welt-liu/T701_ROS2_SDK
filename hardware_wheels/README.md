# Native motor SDK wheels / 原生电机库

仅真机模式需要；模拟驱动、模型显示和 QP 求解不需要安装。使用 ROS 2 Humble 和 Python 3.10，在 SDK 根目录按电脑架构选择一条命令。

Hardware mode requires the native motor SDK. Simulation, model display, and QP do not. Use ROS 2 Humble with Python 3.10 and run the matching command from the SDK root.

Linux x86_64:

```bash
.venv/bin/python -m pip install hardware_wheels/hightorque_robot-1.2.0-cp310-cp310-linux_x86_64.whl
```

Linux ARM64 / aarch64:

```bash
.venv/bin/python -m pip install hardware_wheels/hightorque_robot-1.0.0-cp310-cp310-linux_aarch64.whl
```

通用 Python 依赖统一由根目录的 `requirements.txt` 记录；模型库使用 README 中的 ROS 系统包安装步骤。不要在 Python 3.13 的 Conda 环境中安装 CPython 3.10 wheel。

Shared Python dependencies are listed in the root `requirements.txt`. Install model libraries through the ROS system packages described in the user guide. CPython 3.10 wheels require Python 3.10.

原版 wheel 的文件校验值见 `SOURCE_HASHES.json`。两个架构的 wheel 版本不同，实机验证范围见根目录 `VALIDATION.md`。

Original wheel checksums are recorded in `SOURCE_HASHES.json`. The bundled wheel versions differ by architecture. Hardware validation scope is recorded in the root `VALIDATION.md`.
