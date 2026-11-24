#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
unset PYTHONPATH PYTHONHOME LD_PRELOAD AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
if [[ ! -x .venv/bin/python ]]; then
  /usr/bin/python3 -m venv --without-pip --system-site-packages .venv
fi
if ! .venv/bin/python -c 'import osqp; assert int(osqp.__version__.split(".")[0]) >= 1' 2>/dev/null; then
  printf '%s\n' 'OSQP >= 1 is needed. See README.md: environment setup.' >&2
  exit 1
fi
.venv/bin/python -c 'import rclpy, pinocchio, scipy, yaml, osqp'
.venv/bin/python /usr/bin/colcon build --base-paths src --event-handlers console_cohesion+
printf '%s\n' 'Build complete. Start RViz with: ./start.sh'
