#!/usr/bin/env python3
"""
MAVLink 드론 제어 스크립트

이 스크립트는 PX4 SITL에 직접 MAVLink 명령을 전송합니다.
Isaac Sim 외부에서 실행 가능합니다.

사용법:
    python3 mavlink_drone_control.py

필요 패키지:
    pip install pymavlink
"""

import time
import math
import argparse
from typing import Tuple, Optional

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("[ERROR] pymavlink not installed. Run: pip install pymavlink")


class MAVLinkDroneController:
    """MAVLink 드론 제어 클래스"""
    
    def __init__(self, connection_string: str = "px4"):
        """
        Args:
            connection_string: MAVLink 연결 문자열
                - "px4" 또는 "sitl": PX4 SITL 전용 (권장)
                - "udpin:0.0.0.0:14540": 수신만
        """
        self.connection_string = connection_string
        self.master = None
        self.cmd_socket = None
        self.cmd_target = ('127.0.0.1', 14580)
        self.home_position = None
        
    def connect(self) -> bool:
        """드론에 연결"""
        if not MAVLINK_AVAILABLE:
            return False
        
        import socket
            
        try:
            print(f"[CONNECT] Connecting to {self.connection_string}...")
            
            # 수신용 연결 (14540에서 PX4 telemetry 수신)
            self.master = mavutil.mavlink_connection(
                'udpin:0.0.0.0:14540',
                source_system=255,
                source_component=0
            )
            print("[CONNECT] Waiting for heartbeat...")
            self.master.wait_heartbeat(timeout=10)
            
            # 송신용 소켓 생성 (14580으로 명령 전송)
            self.cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            print(f"[CONNECT] Connected! System ID: {self.master.target_system}")
            print(f"[CONNECT] Commands will be sent to {self.cmd_target}")
            return True
        except Exception as e:
            print(f"[ERROR] Connection failed: {e}")
            return False
    
    def _send_msg(self, msg):
        """MAVLink 메시지를 14580 포트로 전송"""
        if self.cmd_socket:
            self.cmd_socket.sendto(msg.pack(self.master.mav), self.cmd_target)
    
    def arm(self) -> bool:
        """드론 시동"""
        if not self.master:
            return False
            
        print("[ARM] Arming drone...")
        msg = self.master.mav.command_long_encode(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0
        )
        self._send_msg(msg)
        
        # ACK 대기
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("[ARM] Armed successfully!")
            return True
        print("[ARM] Arm command sent (no ACK)")
        return True  # ACK 없어도 진행
    
    def disarm(self) -> bool:
        """드론 시동 해제"""
        if not self.master:
            return False
            
        print("[DISARM] Disarming drone...")
        msg = self.master.mav.command_long_encode(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # disarm
            0, 0, 0, 0, 0, 0
        )
        self._send_msg(msg)
        return True
    
    def takeoff(self, altitude: float = 3.0) -> bool:
        """이륙"""
        if not self.master:
            return False
            
        print(f"[TAKEOFF] Taking off to {altitude}m...")
        msg = self.master.mav.command_long_encode(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude
        )
        self._send_msg(msg)
        
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"[TAKEOFF] Takeoff command accepted!")
            return True
        print("[TAKEOFF] Takeoff command sent (no ACK)")
        return True  # ACK 없어도 진행
    
    def launch(self, altitude: float = 3.0, duration: float = 5.0) -> bool:
        """
        OFFBOARD 모드로 이륙 (arm + setpoint + offboard 모드)
        가장 안정적인 이륙 방식
        """
        import time
        
        if not self.master:
            return False
        
        print(f"[LAUNCH] OFFBOARD takeoff to {altitude}m...")
        
        # 1. Arm
        arm_msg = self.master.mav.command_long_encode(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        self._send_msg(arm_msg)
        print("[LAUNCH] Armed")
        
        # 2. setpoint 전송 시작 (OFFBOARD 진입 전 필수)
        # 현재 위치에서 altitude만큼 위로
        ned_down = -altitude
        
        print(f"[LAUNCH] Sending setpoints + switching to OFFBOARD...")
        
        start_time = time.time()
        offboard_sent = False
        
        while time.time() - start_time < duration:
            # setpoint 전송
            msg = self.master.mav.set_position_target_local_ned_encode(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                0, 0, ned_down,  # 현재 위치 위로
                0, 0, 0,
                0, 0, 0,
                0, 0
            )
            self._send_msg(msg)
            
            # 1초 후 OFFBOARD 모드 전환
            if not offboard_sent and time.time() - start_time > 1.0:
                mode_msg = self.master.mav.command_long_encode(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    14,  # OFFBOARD mode ID
                    0, 0, 0, 0, 0
                )
                self._send_msg(mode_msg)
                print("[LAUNCH] OFFBOARD mode requested")
                offboard_sent = True
            
            time.sleep(0.05)  # 20Hz
        
        print(f"[LAUNCH] Done! Hovering at {altitude}m")
        return True
    
    def land(self) -> bool:
        """착륙"""
        if not self.master:
            return False
            
        print("[LAND] Landing...")
        msg = self.master.mav.command_long_encode(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        self._send_msg(msg)
        return True
    
    def set_mode(self, mode: str) -> bool:
        """비행 모드 설정"""
        if not self.master:
            return False
        
        mode_mapping = {
            'STABILIZED': 'STABILIZED',
            'OFFBOARD': 'OFFBOARD',
            'POSCTL': 'POSCTL',
            'ALTCTL': 'ALTCTL',
            'MANUAL': 'MANUAL',
            'AUTO.LOITER': 'AUTO.LOITER',
            'AUTO.RTL': 'AUTO.RTL',
            'AUTO.LAND': 'AUTO.LAND',
        }
        
        if mode.upper() not in mode_mapping:
            print(f"[MODE] Unknown mode: {mode}")
            return False
        
        mode_name = mode_mapping[mode.upper()]
        print(f"[MODE] Setting mode to {mode_name}...")
        
        # PX4 custom mode
        msg = self.master.mav.command_long_encode(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self._get_px4_mode_id(mode_name),
            0, 0, 0, 0, 0
        )
        self._send_msg(msg)
        
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"[MODE] Mode set to {mode_name}!")
            return True
        print(f"[MODE] Mode command sent (no ACK)")
        return True  # ACK 없어도 진행
    
    def _get_px4_mode_id(self, mode_name: str) -> int:
        """PX4 모드 ID 반환"""
        # PX4 custom mode IDs
        modes = {
            'MANUAL': 1,
            'ALTCTL': 2,
            'POSCTL': 3,
            'AUTO.MISSION': 4,
            'AUTO.LOITER': 5,
            'AUTO.RTL': 6,
            'STABILIZED': 7,
            'OFFBOARD': 14,
            'AUTO.LAND': 9,
        }
        return modes.get(mode_name, 0)
    
    def offboard_mode(self) -> bool:
        """OFFBOARD 모드로 전환 (goto 명령 사용 전 필수)"""
        return self.set_mode('OFFBOARD')
    
    def goto_position_ned(self, north: float, east: float, down: float, yaw: float = 0) -> bool:
        """
        NED 좌표로 이동 (로컬 좌표, 홈 위치 기준)
        
        Args:
            north: 북쪽 거리 (m)
            east: 동쪽 거리 (m)
            down: 아래쪽 거리 (m, 음수 = 위로)
            yaw: 방향 (rad)
        """
        if not self.master:
            return False
        
        print(f"[GOTO] Moving to NED: N={north:.1f}, E={east:.1f}, D={down:.1f}")
        
        # SET_POSITION_TARGET_LOCAL_NED 메시지
        msg = self.master.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask (positions only)
            north, east, down,  # position
            0, 0, 0,  # velocity (ignored)
            0, 0, 0,  # acceleration (ignored)
            yaw,  # yaw
            0     # yaw_rate
        )
        self._send_msg(msg)
        return True
    
    def goto_isaac_coords(self, x: float, y: float, z: float, yaw: float = 0) -> bool:
        """
        Isaac Sim 좌표로 이동 (단일 setpoint)
        
        Isaac Sim → PX4 NED 좌표 변환:
            Isaac X → NED East
            Isaac Y → NED North
            Isaac Z → NED Down (반전)
        
        Args:
            x: Isaac Sim X 좌표
            y: Isaac Sim Y 좌표
            z: Isaac Sim Z 좌표 (고도)
            yaw: 방향 (rad)
        """
        ned_north = y
        ned_east = x
        ned_down = -z
        
        print(f"[GOTO] Isaac Sim coords: ({x:.1f}, {y:.1f}, {z:.1f})")
        return self.goto_position_ned(ned_north, ned_east, ned_down, yaw)
    
    def fly_to(self, x: float, y: float, z: float, duration: float = 5.0) -> bool:
        """
        Isaac Sim 좌표로 비행 (OFFBOARD 모드 + 연속 setpoint)
        
        Args:
            x, y, z: Isaac Sim 좌표
            duration: 비행 시간 (초)
        """
        import time
        
        if not self.master:
            return False
        
        ned_north = y
        ned_east = x
        ned_down = -z
        
        print(f"[FLY] Flying to Isaac Sim ({x:.1f}, {y:.1f}, {z:.1f}) for {duration}s...")
        print(f"[FLY] NED target: N={ned_north:.1f}, E={ned_east:.1f}, D={ned_down:.1f}")
        
        start_time = time.time()
        count = 0
        offboard_sent = False
        
        while time.time() - start_time < duration:
            # setpoint 전송
            msg = self.master.mav.set_position_target_local_ned_encode(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                ned_north, ned_east, ned_down,
                0, 0, 0,
                0, 0, 0,
                0, 0
            )
            self._send_msg(msg)
            count += 1
            
            # 처음 0.5초에 OFFBOARD 모드 전환 요청
            if not offboard_sent and time.time() - start_time > 0.5:
                mode_msg = self.master.mav.command_long_encode(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    14,  # OFFBOARD
                    0, 0, 0, 0, 0
                )
                self._send_msg(mode_msg)
                offboard_sent = True
            
            time.sleep(0.05)  # 20Hz
        
        print(f"[FLY] Sent {count} setpoints. Done.")
        return True
    
    def get_position(self) -> Optional[Tuple[float, float, float]]:
        """현재 위치 반환 (NED)"""
        if not self.master:
            return None
        
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
        if msg:
            return (msg.x, msg.y, msg.z)
        return None
    
    def get_attitude(self) -> Optional[Tuple[float, float, float]]:
        """현재 자세 반환 (roll, pitch, yaw in rad)"""
        if not self.master:
            return None
        
        msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
        if msg:
            return (msg.roll, msg.pitch, msg.yaw)
        return None
    
    def print_status(self):
        """드론 상태 출력"""
        pos = self.get_position()
        att = self.get_attitude()
        
        print("\n[STATUS]")
        if pos:
            # NED → Isaac Sim 좌표 변환
            isaac_x = pos[1]  # NED East → Isaac X
            isaac_y = pos[0]  # NED North → Isaac Y
            isaac_z = -pos[2]  # NED Down → Isaac Z
            print(f"  Position (NED):       N={pos[0]:.2f}, E={pos[1]:.2f}, D={pos[2]:.2f}")
            print(f"  Position (Isaac Sim): X={isaac_x:.2f}, Y={isaac_y:.2f}, Z={isaac_z:.2f}")
        if att:
            print(f"  Attitude: roll={math.degrees(att[0]):.1f}°, pitch={math.degrees(att[1]):.1f}°, yaw={math.degrees(att[2]):.1f}°")


def run_factory_tour():
    """공장 순회 미션 실행"""
    
    # 웨이포인트 정의 (Isaac Sim 좌표)
    waypoints = [
        (0, 0, 3, "Start"),
        (0, 10, 3, "Entrance"),
        (20, 15, 4, "Center"),
        (15, 30, 4, "Motor_Area_North"),
        (38, 40, 3, "Motor_Area_East"),
        (45, 42, 4, "NE_Corner"),
        (45, 24, 4, "East_Zone"),
        (0, 0, 3, "Return_Home"),
    ]
    
    print("\n" + "=" * 60)
    print("  공장 순회 미션")
    print("=" * 60)
    
    # 드론 연결 (Docker 내부: UDP 14550 직접 사용)
    drone = MAVLinkDroneController("udp:127.0.0.1:14550")
    if not drone.connect():
        print("[ERROR] Cannot connect to drone")
        return
    
    # 상태 확인
    drone.print_status()
    
    # 미션 실행
    input("\n[MISSION] Press Enter to start mission...")
    
    # 시동 및 이륙
    drone.arm()
    time.sleep(1)
    drone.takeoff(3.0)
    time.sleep(5)  # 이륙 대기
    
    # 웨이포인트 순회
    for x, y, z, name in waypoints:
        print(f"\n[WAYPOINT] Going to: {name} ({x}, {y}, {z})")
        drone.goto_isaac_coords(x, y, z)
        time.sleep(5)  # 이동 대기
        drone.print_status()
    
    # 착륙
    print("\n[MISSION] Landing...")
    drone.land()
    time.sleep(5)
    drone.disarm()
    
    print("\n[MISSION] Complete!")


def interactive_mode():
    """대화형 모드"""
    
    print("\n" + "=" * 60)
    print("  드론 대화형 제어")
    print("=" * 60)
    print("""
명령어:
  connect           - 드론 연결 (PX4 SITL: 14540 수신, 14580 송신)
  arm               - 시동
  disarm            - 시동 해제
  takeoff <alt>     - 이륙 (고도 m)
  launch <alt>      - 시동 + 이륙 한 번에 (권장!)
  offboard          - OFFBOARD 모드 전환 (goto 전 필수!)
  goto <x> <y> <z>  - Isaac Sim 좌표로 이동 (단일 setpoint)
  fly <x> <y> <z> [초] - 해당 위치로 비행 (연속 setpoint, 기본 5초)
  land              - 착륙
  status            - 상태 확인
  mode <mode>       - 비행 모드 변경 (POSCTL, OFFBOARD, etc.)
  tour              - 공장 순회 미션
  quit              - 종료
""")
    
    drone = MAVLinkDroneController()
    
    while True:
        try:
            cmd = input("\ndrone> ").strip().lower().split()
            if not cmd:
                continue
            
            if cmd[0] == "quit" or cmd[0] == "exit":
                break
            elif cmd[0] == "connect":
                drone.connect()  # 항상 PX4 SITL 모드 사용
            elif cmd[0] == "arm":
                drone.arm()
            elif cmd[0] == "disarm":
                drone.disarm()
            elif cmd[0] == "takeoff":
                alt = float(cmd[1]) if len(cmd) > 1 else 3.0
                drone.takeoff(alt)
            elif cmd[0] == "launch":
                alt = float(cmd[1]) if len(cmd) > 1 else 3.0
                drone.launch(alt)
            elif cmd[0] == "land":
                drone.land()
            elif cmd[0] == "offboard":
                drone.offboard_mode()
            elif cmd[0] == "mode":
                if len(cmd) >= 2:
                    drone.set_mode(cmd[1])
                else:
                    print("Usage: mode <OFFBOARD|POSCTL|ALTCTL|MANUAL|AUTO.LOITER|AUTO.RTL>")
            elif cmd[0] == "goto":
                if len(cmd) >= 4:
                    x, y, z = float(cmd[1]), float(cmd[2]), float(cmd[3])
                    drone.goto_isaac_coords(x, y, z)
                else:
                    print("Usage: goto <x> <y> <z>")
            elif cmd[0] == "fly":
                if len(cmd) >= 4:
                    x, y, z = float(cmd[1]), float(cmd[2]), float(cmd[3])
                    duration = float(cmd[4]) if len(cmd) >= 5 else 5.0
                    drone.fly_to(x, y, z, duration)
                else:
                    print("Usage: fly <x> <y> <z> [duration_seconds]")
            elif cmd[0] == "status":
                drone.print_status()
            elif cmd[0] == "tour":
                run_factory_tour()
            else:
                print(f"Unknown command: {cmd[0]}")
                
        except KeyboardInterrupt:
            print("\nInterrupted")
            break
        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MAVLink Drone Controller")
    parser.add_argument("--tour", action="store_true", help="Run factory tour mission")
    parser.add_argument("--connect", default="udp:127.0.0.1:14550", help="MAVLink connection string (Docker 내부: udp:127.0.0.1:14550)")
    args = parser.parse_args()
    
    if args.tour:
        run_factory_tour()
    else:
        interactive_mode()
