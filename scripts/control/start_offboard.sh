#!/bin/bash
# =============================================================================
# ROS2 OFFBOARD Control Startup Script
# =============================================================================
#
# This script helps you set up and run the ROS2 OFFBOARD control for PX4 SITL
# in Isaac Sim + Pegasus.
#
# Usage:
#   ./start_offboard.sh [agent|control|all]
#
#   agent   - Start MicroXRCEAgent only
#   control - Start the OFFBOARD control script
#   all     - Start both (default)
#
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║       🚁 ROS2 OFFBOARD CONTROL SETUP - PX4 + PEGASUS 🚁     ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_step() {
    echo -e "${GREEN}[STEP]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        print_warning "ROS2 환경이 활성화되지 않았습니다."
        print_step "ROS2 환경 활성화 중..."
        
        if [ -f /opt/ros/humble/setup.bash ]; then
            source /opt/ros/humble/setup.bash
            echo -e "${GREEN}✅ ROS2 Humble 활성화됨${NC}"
        else
            print_error "ROS2 Humble을 찾을 수 없습니다!"
            exit 1
        fi
        
        if [ -f /root/ros2_ws/install/setup.bash ]; then
            source /root/ros2_ws/install/setup.bash
            echo -e "${GREEN}✅ px4_msgs 워크스페이스 활성화됨${NC}"
        else
            print_warning "px4_msgs 워크스페이스를 찾을 수 없습니다."
        fi
        
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    else
        echo -e "${GREEN}✅ ROS2 $ROS_DISTRO 환경 이미 활성화됨${NC}"
    fi
}

check_agent_running() {
    if pgrep -x "MicroXRCEAgent" > /dev/null; then
        return 0
    else
        return 1
    fi
}

start_agent() {
    print_step "MicroXRCEAgent 상태 확인 중..."
    
    if check_agent_running; then
        echo -e "${GREEN}✅ MicroXRCEAgent가 이미 실행 중입니다.${NC}"
    else
        print_step "MicroXRCEAgent 시작 중..."
        MicroXRCEAgent udp4 -p 8888 &
        AGENT_PID=$!
        sleep 2
        
        if check_agent_running; then
            echo -e "${GREEN}✅ MicroXRCEAgent 시작됨 (PID: $AGENT_PID)${NC}"
        else
            print_error "MicroXRCEAgent 시작 실패!"
            exit 1
        fi
    fi
}

start_control() {
    check_ros2
    
    print_step "OFFBOARD 제어 스크립트 시작 중..."
    echo ""
    
    python3 "$SCRIPT_DIR/ros2_offboard_control.py"
}

show_usage() {
    echo "사용법: $0 [agent|control|all]"
    echo ""
    echo "  agent   - MicroXRCEAgent만 시작"
    echo "  control - OFFBOARD 제어 스크립트만 시작"
    echo "  all     - 둘 다 시작 (기본값)"
    echo ""
    echo "전제 조건:"
    echo "  1. Isaac Sim + Pegasus가 PX4 SITL과 함께 실행 중이어야 합니다."
    echo "  2. Docker 컨테이너 내부에서 실행해야 합니다."
}

# Main
print_header

case "${1:-all}" in
    agent)
        start_agent
        echo ""
        echo -e "${YELLOW}💡 이제 다른 터미널에서 다음을 실행하세요:${NC}"
        echo "   ros2_env"
        echo "   python3 $SCRIPT_DIR/ros2_offboard_control.py"
        ;;
    control)
        start_control
        ;;
    all)
        start_agent
        echo ""
        start_control
        ;;
    help|--help|-h)
        show_usage
        ;;
    *)
        print_error "알 수 없는 옵션: $1"
        show_usage
        exit 1
        ;;
esac
