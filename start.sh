#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ $# -eq 0 ]]; then
  set -- sim
fi
mode=""
if [[ "${1:-}" == sim || "${1:-}" == real ]]; then
  mode="$1"
  shift
  sim_domain="${T701_HT_SIM_DOMAIN_ID:-20}"
  real_domain="${T701_HT_REAL_DOMAIN_ID:-21}"
  for domain in "$sim_domain" "$real_domain"; do
    if [[ ! "$domain" =~ ^[0-9]{1,3}$ ]] || (( 10#$domain > 232 )); then
      printf '%s\n' 'T701-HT domain IDs must be integers from 0 to 232.' >&2
      exit 2
    fi
  done
  if (( 10#$sim_domain == 10#$real_domain )); then
    printf '%s\n' 'Simulation and hardware must use different domain IDs.' >&2
    exit 2
  fi
  if [[ "$mode" == sim ]]; then
    export ROS_DOMAIN_ID="$sim_domain"
    simulate=true
  else
    export ROS_DOMAIN_ID="$real_domain"
    simulate=false
  fi
  printf 'T701-HT: %s | ROS_DOMAIN_ID=%s\n' "$mode" "$ROS_DOMAIN_ID"
  if [[ $# -eq 0 || "$1" == *:=* ]]; then
    for option in "$@"; do
      case "$option" in
        arms:=*|rviz:=*|state_rate:=*|enable_on_start:=*) ;;
        *) printf 'Unsupported startup option: %s\n' "$option" >&2; exit 2 ;;
      esac
    done
    set -- launch t701_ht_ros2 sdk.launch.py "simulate:=$simulate" rviz:=true "$@"
  elif [[ "$1" == demo ]]; then
    shift
    for option in "$@"; do
      case "$option" in
        --target|--target=*) printf '%s\n' 'The sim/real entry selects --target automatically.' >&2; exit 2 ;;
      esac
    done
    set -- run t701_ht_ros2 demo --target "$mode" "$@"
  else
    case "$1" in
      topic|service|action|node|param|interface) ;;
      run)
        if [[ "${2:-}" != rviz2 || "${3:-}" != rviz2 ]]; then
          printf '%s\n' 'Use: start.sh sim|real demo [options], or run rviz2 rviz2.' >&2
          exit 2
        fi ;;
      *) printf '%s\n' 'Use: start.sh sim|real [arms:=...] or sim|real demo [options].' >&2; exit 2 ;;
    esac
  fi
fi
unset PYTHONPATH PYTHONHOME LD_PRELOAD AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
if [[ ! -f install/setup.bash ]]; then
  printf '%s\n' 'Run ./build.sh before starting.' >&2
  exit 1
fi
source install/setup.bash
# The built entry points use this workspace's Python 3.10 environment.
export PATH="$PWD/.venv/bin:$PATH"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/log/runtime}"
mkdir -p -- "$ROS_LOG_DIR"
if [[ "${T701_HT_SOFTWARE_RENDERING:-0}" == 1 ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa QT_X11_NO_MITSHM=1
fi
if [[ $# -eq 0 ]]; then
  exec .venv/bin/python /opt/ros/humble/bin/ros2 launch t701_ht_ros2 display.launch.py
fi
exec .venv/bin/python /opt/ros/humble/bin/ros2 "$@"
