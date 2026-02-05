#!/bin/bash
# =============================================================================
# 호스트에서 실행: Docker 컨테이너 안에서 ROS2 OFFBOARD 제어 스크립트 실행
# =============================================================================
#
# 사용법:
#   ./run_ros2_offboard.sh
#   ./run_ros2_offboard.sh <컨테이너_이름>
#   CONTAINER=my_container ./run_ros2_offboard.sh
#
# 전제: 프로젝트가 컨테이너에 마운트되어 있어야 합니다.
#       마운트 경로가 다르면 아래 SCRIPT_DIR_IN_CONTAINER 를 수정하세요.
# =============================================================================

CONTAINER="${CONTAINER:-isaac-pegasus-headless}"
# 컨테이너 내부에서 스크립트 경로 (마운트 위치에 맞게 수정)
SCRIPT_DIR_IN_CONTAINER="${SCRIPT_DIR_IN_CONTAINER:-/scratch/minsuh/my_isaac_project/scripts/control}"

if [ -n "$1" ] && [ "$1" != "--" ]; then
    CONTAINER="$1"
fi

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "[ERROR] 컨테이너 '$CONTAINER'가 실행 중이지 않습니다."
    echo "  실행 중인 컨테이너: $(docker ps --format '{{.Names}}' 2>/dev/null | tr '\n' ' ')"
    exit 1
fi

echo "컨테이너: $CONTAINER"
echo "스크립트 경로: $SCRIPT_DIR_IN_CONTAINER"
echo ""

docker exec -it "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  source /root/ros2_ws/install/setup.bash 2>/dev/null || true
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  if [ -f '$SCRIPT_DIR_IN_CONTAINER/ros2_offboard_control.py' ]; then
    cd '$SCRIPT_DIR_IN_CONTAINER' && python3 ros2_offboard_control.py
  else
    echo '[ERROR] 컨테이너 내부에 스크립트가 없습니다: $SCRIPT_DIR_IN_CONTAINER/ros2_offboard_control.py'
    echo '  프로젝트를 컨테이너에 마운트했는지, SCRIPT_DIR_IN_CONTAINER 경로가 맞는지 확인하세요.'
    exit 1
  fi
"
