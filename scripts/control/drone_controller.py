#!/usr/bin/env python3
"""
드론 제어 스크립트 (Pegasus Simulator API)

사용법:
1. Isaac Sim에서 Script Editor 열기 (Window → Script Editor)
2. 이 스크립트 내용을 붙여넣기
3. Run 클릭

또는 Isaac Sim Python 환경에서 실행:
    ./python.sh scripts/control/drone_controller.py
"""

import asyncio
import numpy as np

# Omniverse/Isaac Sim imports
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage

# Pegasus imports
try:
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
    PEGASUS_AVAILABLE = True
except ImportError:
    PEGASUS_AVAILABLE = False
    print("[WARNING] Pegasus Simulator not found. Some features may not work.")


class DroneController:
    """드론 제어 클래스"""
    
    def __init__(self):
        self.vehicle = None
        self.world = None
        self._setup()
    
    def _setup(self):
        """World 및 드론 참조 설정"""
        # 현재 World 가져오기
        self.world = World.instance()
        if self.world is None:
            print("[ERROR] World not initialized. Please run simulation first.")
            return
        
        # Stage에서 드론 찾기
        stage = get_current_stage()
        if stage is None:
            print("[ERROR] No stage found.")
            return
        
        # quadrotor prim 찾기
        drone_path = "/World/quadrotor"
        prim = stage.GetPrimAtPath(drone_path)
        if prim.IsValid():
            print(f"[INFO] Found drone at: {drone_path}")
        else:
            print(f"[WARNING] Drone not found at {drone_path}")
            # 다른 경로 시도
            for path in ["/quadrotor", "/root/quadrotor"]:
                prim = stage.GetPrimAtPath(path)
                if prim.IsValid():
                    drone_path = path
                    print(f"[INFO] Found drone at: {drone_path}")
                    break
    
    def get_drone_position(self):
        """현재 드론 위치 반환 (Isaac Sim 좌표)"""
        stage = get_current_stage()
        drone_prim = stage.GetPrimAtPath("/World/quadrotor")
        
        if not drone_prim.IsValid():
            print("[ERROR] Drone prim not found")
            return None
        
        from pxr import UsdGeom
        xform = UsdGeom.Xformable(drone_prim)
        transform = xform.ComputeLocalToWorldTransform(0)
        position = transform.ExtractTranslation()
        
        return np.array([position[0], position[1], position[2]])
    
    def print_drone_status(self):
        """드론 상태 출력"""
        pos = self.get_drone_position()
        if pos is not None:
            print(f"[DRONE STATUS]")
            print(f"  Position (X, Y, Z): ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            print(f"  Factory coordinates:")
            print(f"    - X range: [-3, 48] → current: {pos[0]:.2f}")
            print(f"    - Y range: [-3, 45] → current: {pos[1]:.2f}")
            print(f"    - Z (altitude): {pos[2]:.2f}m")


class WaypointMission:
    """웨이포인트 미션 클래스"""
    
    def __init__(self):
        self.waypoints = []
        self.current_index = 0
    
    def add_waypoint(self, x, y, z, name=""):
        """웨이포인트 추가 (Isaac Sim 좌표)"""
        self.waypoints.append({
            'position': np.array([x, y, z]),
            'name': name or f"WP_{len(self.waypoints)}"
        })
        print(f"[MISSION] Added waypoint: {name or f'WP_{len(self.waypoints)-1}'} at ({x}, {y}, {z})")
    
    def create_factory_tour(self):
        """공장 순회 미션 생성"""
        # 커스텀 Factory 맵 기준 웨이포인트
        self.waypoints = []
        
        # 시작 위치 (원점 근처)
        self.add_waypoint(0, 0, 3, "Start")
        
        # 입구 영역
        self.add_waypoint(0, 10, 3, "Entrance")
        
        # 중앙 구역
        self.add_waypoint(20, 15, 4, "Center")
        
        # 모터 영역 (Camera_03 근처)
        self.add_waypoint(15, 30, 4, "Motor_Area_North")
        
        # 동쪽 모터 영역
        self.add_waypoint(38, 40, 3, "Motor_Area_East")
        
        # 북동 코너 (Camera_05 근처)
        self.add_waypoint(45, 42, 4, "NE_Corner")
        
        # 동쪽 구역 (Camera_04 근처)  
        self.add_waypoint(45, 24, 4, "East_Zone")
        
        # 복귀
        self.add_waypoint(0, 0, 3, "Return_Home")
        
        print(f"[MISSION] Factory tour created with {len(self.waypoints)} waypoints")
        return self.waypoints
    
    def get_next_waypoint(self):
        """다음 웨이포인트 반환"""
        if self.current_index < len(self.waypoints):
            wp = self.waypoints[self.current_index]
            self.current_index += 1
            return wp
        return None
    
    def print_mission(self):
        """미션 출력"""
        print("\n[MISSION PLAN]")
        print("=" * 50)
        for i, wp in enumerate(self.waypoints):
            marker = ">>>" if i == self.current_index else "   "
            pos = wp['position']
            print(f"{marker} {i+1}. {wp['name']}: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
        print("=" * 50)


def send_mavlink_goto(x, y, z, yaw=0):
    """
    MAVLink를 통해 드론에 goto 명령 전송
    
    Note: 이 함수는 PX4 SITL이 실행 중일 때 작동합니다.
    Isaac Sim 좌표 → PX4 NED 좌표 변환 포함
    """
    # Isaac Sim (X, Y, Z) → PX4 NED (North, East, Down)
    # X → East (Y in NED)
    # Y → North (X in NED)  
    # Z → Up (-Down in NED)
    
    ned_north = y  # Isaac Y → NED North
    ned_east = x   # Isaac X → NED East
    ned_down = -z  # Isaac Z → NED Down (반전)
    
    print(f"[MAVLINK] Sending goto command:")
    print(f"  Isaac Sim: ({x:.2f}, {y:.2f}, {z:.2f})")
    print(f"  PX4 NED:   (N:{ned_north:.2f}, E:{ned_east:.2f}, D:{ned_down:.2f})")
    
    # MAVLink 명령 (실제 구현은 pymavlink 필요)
    command = f"commander goto {ned_north} {ned_east} {ned_down} {yaw}"
    print(f"  Command: {command}")
    
    return command


# ============================================================
# 메인 실행 (Isaac Sim Script Editor에서 실행)
# ============================================================

def main():
    """메인 함수 - Script Editor에서 실행"""
    print("\n" + "=" * 60)
    print("  드론 제어 스크립트 시작")
    print("=" * 60 + "\n")
    
    # 드론 컨트롤러 초기화
    controller = DroneController()
    controller.print_drone_status()
    
    # 웨이포인트 미션 생성
    mission = WaypointMission()
    mission.create_factory_tour()
    mission.print_mission()
    
    # 첫 번째 웨이포인트로 이동 명령 출력
    print("\n[COMMANDS] 아래 명령을 QGC MAVLink Console에 입력하세요:\n")
    
    for wp in mission.waypoints:
        pos = wp['position']
        cmd = send_mavlink_goto(pos[0], pos[1], pos[2])
        print(f"  # {wp['name']}")
        print(f"  {cmd}\n")


# Script Editor에서 실행 시
if __name__ == "__main__":
    main()
