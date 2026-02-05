#!/usr/bin/env python3
"""
ROS2 OFFBOARD Control for PX4 SITL in Isaac Sim + Pegasus

This script provides position control for drones using ROS2 and PX4's OFFBOARD mode.

Prerequisites:
    1. Isaac Sim + Pegasus running with PX4 SITL
    2. MicroXRCEAgent running: MicroXRCEAgent udp4 -p 8888
    3. ROS2 environment sourced: ros2_env

Usage:
    # Terminal 1: Start MicroXRCEAgent
    MicroXRCEAgent udp4 -p 8888
    
    # Terminal 2: Run this script
    ros2_env
    python3 ros2_offboard_control.py
    
Commands:
    connect     - Connect to drone (start heartbeat)
    arm         - Arm the drone
    disarm      - Disarm the drone
    takeoff [h] - Take off to height h meters (default: 3m)
    land        - Land the drone
    fly x y z   - Fly to position (x, y, z) in meters
    hover       - Hold current position
    status      - Show current drone status
    help        - Show this help
    quit        - Exit the program

Author: Generated for Isaac Sim + Pegasus + PX4 SITL
"""

import sys

# ROS2(rclpy)는 Docker 컨테이너 내부에서 ros2_env 후에만 사용 가능합니다.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rclpy.clock import Clock
except ModuleNotFoundError as e:
    print("""
[ERROR] rclpy를 찾을 수 없습니다.

이 스크립트는 ROS2 환경이 있는 곳에서만 실행할 수 있습니다.

▶ Docker 컨테이너 **안**에서 실행하세요:

   docker exec -it isaac-pegasus-headless /bin/bash
   cd /scratch/minsuh/my_isaac_project/scripts/control   # 또는 마운트된 경로
   ros2_env
   python3 ros2_offboard_control.py

▶ 또는 호스트에서 한 번에 실행:

   ./run_ros2_offboard.sh

(컨테이너 이름이 다르면 run_ros2_offboard.sh 내용을 수정하세요)
""", file=sys.stderr)
    sys.exit(1)

import threading
import time

# PX4 message types
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleOdometry
)


class OffboardController(Node):
    """ROS2 Node for OFFBOARD control of PX4 drones."""

    def __init__(self):
        super().__init__('offboard_controller')
        
        # QoS Profile for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback, qos_profile)

        # State variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_odometry = VehicleOdometry()
        
        self.connected = False
        self.armed = False
        self.offboard_mode = False
        self.heartbeat_count = 0
        
        # Target setpoint (NED coordinates: North, East, Down)
        # Note: Z is negative for altitude (down is positive in NED)
        self.target_position = [0.0, 0.0, -3.0]  # Default: hover at 3m
        self.target_yaw = 0.0
        
        # Heartbeat timer (10Hz - publishes control mode and setpoints)
        self.heartbeat_timer = None
        self.heartbeat_active = False
        
        self.get_logger().info('OffboardController initialized')

    def vehicle_local_position_callback(self, msg):
        """Callback for vehicle local position."""
        self.vehicle_local_position = msg
        if not self.connected and msg.xy_valid and msg.z_valid:
            self.connected = True
            self.get_logger().info('✅ Drone connected! Position data received.')

    def vehicle_status_callback(self, msg):
        """Callback for vehicle status."""
        self.vehicle_status = msg
        
        # Update armed state
        new_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if new_armed != self.armed:
            self.armed = new_armed
            status = "ARMED" if self.armed else "DISARMED"
            self.get_logger().info(f'🔔 Drone is now {status}')
        
        # Check if in offboard mode
        self.offboard_mode = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def vehicle_odometry_callback(self, msg):
        """Callback for vehicle odometry."""
        self.vehicle_odometry = msg

    def start_heartbeat(self):
        """Start the heartbeat timer for OFFBOARD control."""
        if self.heartbeat_timer is None:
            # 10Hz timer for publishing control mode and setpoints
            self.heartbeat_timer = self.create_timer(0.1, self.heartbeat_callback)
            self.heartbeat_active = True
            self.get_logger().info('💓 Heartbeat started (10Hz)')

    def stop_heartbeat(self):
        """Stop the heartbeat timer."""
        if self.heartbeat_timer is not None:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None
            self.heartbeat_active = False
            self.get_logger().info('💔 Heartbeat stopped')

    def heartbeat_callback(self):
        """Publish heartbeat: control mode and trajectory setpoint."""
        self.heartbeat_count += 1
        
        # Publish offboard control mode
        self.publish_offboard_control_mode()
        
        # Publish trajectory setpoint
        self.publish_trajectory_setpoint(
            self.target_position[0],
            self.target_position[1],
            self.target_position[2],
            self.target_yaw
        )

    def publish_offboard_control_mode(self):
        """Publish offboard control mode message."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Publish trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command: int, **kwargs):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(kwargs.get('param1', 0.0))
        msg.param2 = float(kwargs.get('param2', 0.0))
        msg.param3 = float(kwargs.get('param3', 0.0))
        msg.param4 = float(kwargs.get('param4', 0.0))
        msg.param5 = float(kwargs.get('param5', 0.0))
        msg.param6 = float(kwargs.get('param6', 0.0))
        msg.param7 = float(kwargs.get('param7', 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        """Arm the drone."""
        self.get_logger().info('🔓 Sending ARM command...')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )

    def disarm(self):
        """Disarm the drone."""
        self.get_logger().info('🔒 Sending DISARM command...')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )

    def engage_offboard_mode(self):
        """Switch to OFFBOARD mode."""
        self.get_logger().info('🎮 Switching to OFFBOARD mode...')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # Custom mode
            param2=6.0   # OFFBOARD mode
        )

    def land(self):
        """Switch to LAND mode."""
        self.get_logger().info('🛬 Sending LAND command...')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def takeoff(self, altitude: float = 3.0):
        """
        Take off to specified altitude using OFFBOARD mode.
        
        Args:
            altitude: Target altitude in meters (positive value)
        """
        self.get_logger().info(f'🚁 Taking off to {altitude}m...')
        
        # Set target position (current XY, target altitude)
        # NED: Z is negative for altitude
        current_x = self.vehicle_local_position.x if self.vehicle_local_position.xy_valid else 0.0
        current_y = self.vehicle_local_position.y if self.vehicle_local_position.xy_valid else 0.0
        
        self.target_position = [current_x, current_y, -altitude]
        
        # Start heartbeat if not already running
        if not self.heartbeat_active:
            self.start_heartbeat()
        
        # Wait for heartbeat to establish (PX4 needs setpoints before OFFBOARD)
        time.sleep(1.0)
        
        # Switch to OFFBOARD mode
        self.engage_offboard_mode()
        time.sleep(0.5)
        
        # Arm the drone
        self.arm()
        
        self.get_logger().info(f'✅ Takeoff initiated to {altitude}m')

    def fly_to(self, x: float, y: float, z: float):
        """
        Fly to specified position (NED coordinates).
        
        Args:
            x: North position (meters)
            y: East position (meters)  
            z: Altitude (positive value, will be converted to NED)
        """
        # Convert positive altitude to NED (negative Z)
        target_z = -abs(z) if z > 0 else z
        
        self.get_logger().info(f'✈️ Flying to position ({x}, {y}, {abs(target_z)}m)...')
        self.target_position = [x, y, target_z]
        
        if not self.heartbeat_active:
            self.start_heartbeat()
            time.sleep(1.0)
            self.engage_offboard_mode()

    def hover(self):
        """Hold current position."""
        if self.vehicle_local_position.xy_valid and self.vehicle_local_position.z_valid:
            self.target_position = [
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z
            ]
            self.get_logger().info(
                f'🔄 Hovering at ({self.target_position[0]:.2f}, '
                f'{self.target_position[1]:.2f}, {-self.target_position[2]:.2f}m)'
            )
        else:
            self.get_logger().warn('⚠️ Position not available for hover')

    def get_status(self) -> str:
        """Get current drone status."""
        pos = self.vehicle_local_position
        status_lines = [
            "\n" + "="*50,
            "📊 DRONE STATUS",
            "="*50,
            f"Connected: {'✅' if self.connected else '❌'}",
            f"Armed: {'✅' if self.armed else '❌'}",
            f"OFFBOARD Mode: {'✅' if self.offboard_mode else '❌'}",
            f"Heartbeat Active: {'✅' if self.heartbeat_active else '❌'}",
            "-"*50,
        ]
        
        if pos.xy_valid and pos.z_valid:
            status_lines.extend([
                "📍 Current Position (NED):",
                f"   X (North): {pos.x:.2f} m",
                f"   Y (East):  {pos.y:.2f} m",
                f"   Z (Down):  {pos.z:.2f} m (Alt: {-pos.z:.2f} m)",
            ])
        else:
            status_lines.append("📍 Position: Not available")
            
        if pos.vx != 0 or pos.vy != 0 or pos.vz != 0:
            status_lines.extend([
                f"🚀 Velocity: ({pos.vx:.2f}, {pos.vy:.2f}, {pos.vz:.2f}) m/s",
            ])
            
        status_lines.extend([
            "-"*50,
            f"🎯 Target Position: ({self.target_position[0]:.2f}, "
            f"{self.target_position[1]:.2f}, {-self.target_position[2]:.2f}m)",
            "="*50,
        ])
        
        return "\n".join(status_lines)


def print_help():
    """Print help message."""
    help_text = """
╔══════════════════════════════════════════════════════════════╗
║           ROS2 OFFBOARD CONTROL FOR PX4 SITL                 ║
╠══════════════════════════════════════════════════════════════╣
║  connect     - Connect to drone (start heartbeat)            ║
║  arm         - Arm the drone                                 ║
║  disarm      - Disarm the drone                              ║
║  takeoff [h] - Take off to height h meters (default: 3m)     ║
║  land        - Land the drone                                ║
║  fly x y z   - Fly to position (x, y, z) in meters           ║
║  hover       - Hold current position                         ║
║  status      - Show current drone status                     ║
║  offboard    - Switch to OFFBOARD mode                       ║
║  help        - Show this help                                ║
║  quit        - Exit the program                              ║
╠══════════════════════════════════════════════════════════════╣
║  Note: Coordinates are in NED (North, East, Down)            ║
║        Altitude should be positive (converted internally)    ║
╚══════════════════════════════════════════════════════════════╝
"""
    print(help_text)


def interactive_mode(controller: OffboardController):
    """Run interactive command mode."""
    print_help()
    
    while rclpy.ok():
        try:
            cmd_input = input("\n🎮 Command> ").strip().lower()
            if not cmd_input:
                continue
                
            parts = cmd_input.split()
            cmd = parts[0]
            
            if cmd == 'quit' or cmd == 'exit' or cmd == 'q':
                print("👋 Exiting...")
                controller.stop_heartbeat()
                break
                
            elif cmd == 'help' or cmd == 'h' or cmd == '?':
                print_help()
                
            elif cmd == 'connect':
                controller.start_heartbeat()
                print("⏳ Waiting for drone connection...")
                
            elif cmd == 'arm':
                controller.arm()
                
            elif cmd == 'disarm':
                controller.disarm()
                
            elif cmd == 'takeoff':
                altitude = 3.0
                if len(parts) > 1:
                    try:
                        altitude = float(parts[1])
                    except ValueError:
                        print("⚠️ Invalid altitude, using default 3m")
                controller.takeoff(altitude)
                
            elif cmd == 'land':
                controller.land()
                
            elif cmd == 'fly':
                if len(parts) < 4:
                    print("⚠️ Usage: fly x y z (e.g., fly 5 0 3)")
                else:
                    try:
                        x = float(parts[1])
                        y = float(parts[2])
                        z = float(parts[3])
                        controller.fly_to(x, y, z)
                    except ValueError:
                        print("⚠️ Invalid coordinates")
                        
            elif cmd == 'hover':
                controller.hover()
                
            elif cmd == 'status':
                print(controller.get_status())
                
            elif cmd == 'offboard':
                controller.engage_offboard_mode()
                
            else:
                print(f"⚠️ Unknown command: {cmd}. Type 'help' for available commands.")
                
        except KeyboardInterrupt:
            print("\n👋 Interrupted. Exiting...")
            controller.stop_heartbeat()
            break
        except EOFError:
            print("\n👋 EOF. Exiting...")
            controller.stop_heartbeat()
            break


def main():
    """Main function."""
    print("""
╔══════════════════════════════════════════════════════════════╗
║       🚁 ROS2 OFFBOARD CONTROL - PX4 SITL + PEGASUS 🚁       ║
╚══════════════════════════════════════════════════════════════╝

Prerequisites:
  1. Isaac Sim + Pegasus running with PX4 SITL
  2. MicroXRCEAgent running: MicroXRCEAgent udp4 -p 8888
  3. ROS2 environment sourced: ros2_env
""")
    
    rclpy.init(args=sys.argv)
    
    controller = OffboardController()
    
    # Start ROS2 spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()
    
    # Give some time for subscriptions to connect
    time.sleep(1.0)
    
    # Run interactive mode
    interactive_mode(controller)
    
    # Cleanup
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
