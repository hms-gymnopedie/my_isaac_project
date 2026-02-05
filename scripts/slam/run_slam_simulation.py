#!/usr/bin/env python3
"""
Drone SLAM Simulation Runner
============================

This script launches Isaac Sim with the Pegasus Simulator extension to run a SLAM experiment.
It loads a specified map (Warehouse or Custom Factory), spawns an Iris drone, and accepts
manual control or ROS 2 offboard control for mapping.

Usage:
    ./python.sh scripts/slam/run_slam_simulation.py --map warehouse
    ./python.sh scripts/slam/run_slam_simulation.py --map custom
"""

import argparse
import sys
import os

# 0. Parse Arguments FIRST (for SimulationApp config)
# We use a separate parser or parse_known_args to avoid conflicts with Kit args later if needed
parser = argparse.ArgumentParser(description="Run Drone SLAM Simulation")
parser.add_argument("--map", type=str, choices=["warehouse", "custom"], default="warehouse", help="Map to load")
parser.add_argument("--headless", action="store_true", help="Run in headless mode (no GUI window)")
parser.add_argument("--webrtc", action="store_true", help="Enable WebRTC streaming")
args, unknown_args = parser.parse_known_args()

# 1. Start SimulationApp FIRST (before other omni imports!)
from omni.isaac.kit import SimulationApp

# Configuration for SimulationApp
# Note: For remote servers, we usually WANT headless=True to avoid X11 errors.
# WebRTC will work even in headless mode if enabled.
config = {
    "headless": args.headless,
    "width": 1280,
    "height": 720,
    "sync_loads": True,
}
kit = SimulationApp(config)

# 2. Imports AFTER SimulationApp (omni modules require SimulationApp to be running)
import carb
import omni
import omni.graph.core as og
import omni.kit.commands
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim, is_prim_path_valid
from omni.isaac.core.utils.extensions import enable_extension
from pxr import Gf, UsdGeom

# Pegasus Imports (Try to import, handle failure)
try:
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
    PEGASUS_AVAILABLE = True
except ImportError:
    carb.log_warn("Pegasus Simulator modules not found. Ensure the extension is enabled.")
    PEGASUS_AVAILABLE = False



def main():
    # Args are already parsed above into 'args'
    
    # 3. Enable Extensions
    # Enable ROS2 bridge (assuming humble)
    enable_extension("omni.isaac.ros2_bridge")
    
    # Enable WebRTC if requested
    if args.webrtc:
        enable_extension("omni.services.streamclient.webrtc")
        enable_extension("omni.kit.livestream.webrtc")
        carb.log_info("WebRTC Streaming Enabled. Connect via browser.")

    # Enable Pegasus
    enable_extension("pegasus.simulator")

    # 4. Initialize World
    world = World(stage_units_in_meters=1.0)

    # 5. Load Map
    if args.map == "warehouse":
        # Load standard Isaac Sim Warehouse
        usd_path = "/Isaac/Environments/Simple_Warehouse/warehouse.usd" 
        # Note: In a real env, verify this path. For now assume standard asset path.
        # If asset server is not connected, this might fail.
        # Alternative: use a local empty stage and build simple walls.
        try:
            # Try open_stage first (for absolute/nucleu paths)
            open_stage(usd_path)
        except Exception:
            # Fallback for testing: Simple Plane
            world.scene.add_default_ground_plane()
            carb.log_warn(f"Could not load {usd_path}. Loaded default ground plane.")
            
    elif args.map == "custom":
        usd_path = "/workspace/assets/environments/260202_my_factory_withCameras.usda"
        if os.path.exists(usd_path):
            open_stage(usd_path)
            carb.log_info(f"Loaded custom map: {usd_path}")
        else:
            carb.log_error(f"Custom map not found at {usd_path}")
            sys.exit(1)



def attach_sensors(drone_path):
    """
    Attach Lidar and Camera to the drone.
    
    Args:
        drone_path (str): Prim path of the drone (e.g., /World/quadrotor)
    """
    stage = omni.usd.get_context().get_stage()
    
    # 1. Attach Lidar
    lidar_path = f"{drone_path}/Lidar"
    if not is_prim_path_valid(lidar_path):
        # Create Lidar Prim
        result, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=lidar_path,
            parent=drone_path,
            config="Example_Rotary", # Use a standard config
            translation=(0, 0, 0.1), # Mount on top
            orientation=Gf.Quatd(1, 0, 0, 0), # Identity
        )
        if result:
            carb.log_info(f"Created Lidar at {lidar_path}")
        else:
            carb.log_error(f"Failed to create Lidar at {lidar_path}")

    # 2. Attach Camera (RGB + Depth)
    camera_path = f"{drone_path}/FrontCamera"
    if not is_prim_path_valid(camera_path):
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Camera",
            prim_path=camera_path,
            attributes={
                "focusDistance": 400,
                "focalLength": 24,
                "clippingRange": (0.01, 10000),
            }
        )
        # Position camera: Facing forward (X-axis)
        # In Isaac Sim, Camera -Z is view direction, so we need to rotate? 
        # Actually standard Camera prim: -Z is look at.
        # Drone Forward is usually +X.
        # So we need to rotate Camera so its -Z aligns with +X. 
        # Rotate -90 deg around Y?
        # Let's place it at front of drone.
        
        # Using UsdGeom to set xform
        prim = stage.GetPrimAtPath(camera_path)
        xform = UsdGeom.Xformable(prim)
        # Translate: 0.1m forward, 0.05m up
        xform.AddTranslateOp().Set(Gf.Vec3d(0.2, 0, 0.05))
        # Rotate: Pitch -90 (to look down) or Standard Front View?
        # If we want front view:
        # Standard Camera: +Y up, -Z view, +X right
        # Drone: +Z up, +X forward, +Y left (or right depending on frame)
        # To align Camera -Z with Drone +X: Rotate 90 deg around Y (so -Z becomes +X? no.)
        # Rotate -90 deg around Y -> -Z becomes +X. +Y stays +Y. +X becomes +Z.
        # But actually in standard robotics frame: Camera Z is forward (optical axis).
        # Wait, USD Camera is NOT optical frame.
        # We will use ROS 2 Bridge Camera Helper which handles optical frame transform usually.
        # Let's just point the camera physically forward.
        xform.AddRotateXYZOp().Set(Gf.Vec3d(0, -90, 0)) # Rotate -90 around Y axis? 
        # Check: Rotation (0, -90, 0)
        # Original: X(1,0,0) Y(0,1,0) Z(0,0,1)
        # Rot -90 Y: X' = Z, Y' = Y, Z' = -X
        # Camera View (-Z') = -(-X) = +X (Drone Forward). Correct.
        # Camera Up (+Y') = +Y (Drone Left/Right?). 
        # If Drone Z is Up, we want Camera Up to be Z?
        # We need Y to be Z.
        # Complex rotation. Let's stick to (0, -90, -90) maybe?
        # Let's just set it roughly forward for now.
        xform.GetRotateXYZOp().Set(Gf.Vec3d(0, -90, -90))
        
        carb.log_info(f"Created Camera at {camera_path}")

    # 3. Setup ROS 2 Bridge (Action Graph)
    create_ros2_bridge_graph(drone_path, lidar_path, camera_path)

def create_ros2_bridge_graph(drone_path, lidar_path, camera_path):
    """
    Create an OmniGraph to bridge Lidar and Camera data to ROS 2.
    """
    keys = omni.graph.core.Controller.Keys
    
    # Create Graph
    graph_path = "/ActionGraph"
    (graph, _, _, _) = omni.graph.core.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                # Lidar Bridge
                ("ReadLidar", "omni.isaac.range_sensor.IsaacReadLidar"), # Note: Node name might vary
                ("PubLaser", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                # Camera Bridge (Using Helper)
                # Note: Helper uses RenderProduct
                ("CreateRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("PubRGB", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("PubDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("PubInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                # TF from PX4/Isaac is usually handled by a separate TF publisher or manually
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "PubLaser.inputs:execIn"),
                ("OnTick.outputs:tick", "PubRGB.inputs:execIn"),
                ("OnTick.outputs:tick", "PubDepth.inputs:execIn"),
                ("OnTick.outputs:tick", "PubInfo.inputs:execIn"),
                ("OnTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "PubLaser.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                # Lidar Settings
                ("ReadLidar.inputs:lidarPrim", lidar_path), # Need to check if this input exists or if we map prim path differently
                ("PubLaser.inputs:topicName", "/scan"),
                ("PubLaser.inputs:frameId", "lidar_link"),
                
                # Camera Settings
                ("CreateRenderProduct.inputs:cameraPrim", camera_path),
                ("CreateRenderProduct.inputs:resolution", (640, 480)),
                
                # RGB Publisher
                ("PubRGB.inputs:checkRenderProductExist", True), # Wait for render product
                ("PubRGB.inputs:frameId", "camera_link"),
                ("PubRGB.inputs:topicName", "/camera/rgb/image_raw"),
                ("PubRGB.inputs:type", "rgb"),
                
                # Depth Publisher
                ("PubDepth.inputs:frameId", "camera_link"),
                ("PubDepth.inputs:topicName", "/camera/depth/image_raw"),
                ("PubDepth.inputs:type", "depth"),
                
                # Info Publisher
                ("PubInfo.inputs:frameId", "camera_link"),
                ("PubInfo.inputs:topicName", "/camera/camera_info"),
                ("PubInfo.inputs:type", "camera_info"),
            ]
        }
    )
    
    # Note: Connecting RenderProduct output to CameraHelper input
    # CreateRenderProduct outputs 'renderProductPath' (token)
    # CameraHelper inputs 'renderProductPath'
    omni.graph.core.Controller.connect(
        f"{graph_path}/CreateRenderProduct.outputs:renderProductPath",
        f"{graph_path}/PubRGB.inputs:renderProductPath"
    )
    omni.graph.core.Controller.connect(
        f"{graph_path}/CreateRenderProduct.outputs:renderProductPath",
        f"{graph_path}/PubDepth.inputs:renderProductPath"
    )
    omni.graph.core.Controller.connect(
        f"{graph_path}/CreateRenderProduct.outputs:renderProductPath",
        f"{graph_path}/PubInfo.inputs:renderProductPath"
    )
    
    carb.log_info("Created ROS 2 Bridge Graph")

    # 6. Spawn Drone (using Pegasus Interface if avail)
    if PEGASUS_AVAILABLE:
        skip_pegasus_spawn = False
        # Create Pegasus Interface to manage vehicles
        pegasus_iface = PegasusInterface()
        
        # Define spawn position
        if args.map == "custom":
            spawn_pos = [0.0, 0.0, 1.0] # Near origin
        else:
            spawn_pos = [0.0, 0.0, 1.0]

        # Use the Pegasus Internal API to spawn a default Iris
        # Example config (simplified)
        drone_config = MultirotorConfig()
        drone_config.vehicle_id = 0
        drone_config.position = spawn_pos
        drone_config.orientation = [1.0, 0.0, 0.0, 0.0] # w, x, y, z
        
        # Create vehicle
        drone = Multirotor(
            "/World/quadrotor",
            drone_config
        )
        
        # Attach Manual Sensors (Placeholder for next step)
        attach_sensors("/World/quadrotor") 

    else:
        carb.log_warn("Pegasus not available. Spawning simple cube as placeholder.")
        world.scene.add_default_ground_plane()
        create_prim("/World/quadrotor", "Cube", position=[0,0,1], scale=[0.5,0.5,0.1])


    # 7. Reset and Run
    world.reset()
    
    # Start Simulation Loop
    while kit.is_running():
        world.step(render=True)
        # Update Pegasus physics if needed (usually handled by extension callbacks)
        # if PEGASUS_AVAILABLE:
        #     pegasus_iface.update() 

    kit.close()

if __name__ == "__main__":
    main()
