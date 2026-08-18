import argparse
import os
import torch
import torch.nn.functional as F

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import omni.kit.app
import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.assets.articulation import Articulation
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sensors import CameraCfg
from isaaclab.managers import SceneEntityCfg

isaaclab_path = "/home/v/IsaacSim/assets"
usd_lite6     = os.path.join(isaaclab_path, "lite6_with_gripper_d435i_v2_abs.usda")
usd_aruco_0   = os.path.join(isaaclab_path, "aruco_0.usd")
usd_aruco_1   = os.path.join(isaaclab_path, "aruco_1.usd")
usd_gear      = os.path.join(isaaclab_path, "gear.usd")
usd_box       = os.path.join(isaaclab_path, "box.usd")

LITE6_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=usd_lite6,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(fix_root_link=True, enabled_self_collisions=True),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0, "finger_joint1": 0.0, "finger_joint2": 0.0},
    ),
    actuators={
        "joint1_2": ImplicitActuatorCfg(joint_names_expr=["joint[12]"],        stiffness=1600.0, damping=80.0, effort_limit=400.0),
        "joint3_5": ImplicitActuatorCfg(joint_names_expr=["joint[345]"],       stiffness=1600.0, damping=51.0, effort_limit=400.0),
        "joint6"  : ImplicitActuatorCfg(joint_names_expr=["joint6"],           stiffness=1600.0, damping=32.0, effort_limit=400.0),
        "gripper" : ImplicitActuatorCfg(joint_names_expr=["finger_joint[12]"], stiffness=800.0,  damping=40.0, effort_limit=5.0),
    },
)

GEAR_CFG = AssetBaseCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=usd_gear, 
        scale=(0.0005, 0.0005, 0.0008),
        
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.6, 0.8)),
        
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_depenetration_velocity=1.0, 
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=16,
        ),
        
        mass_props=sim_utils.MassPropertiesCfg(mass=0.05),
        
        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    ),
    init_state=AssetBaseCfg.InitialStateCfg(),
)

BOX_CFG = AssetBaseCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=usd_box,
        scale=(0.0032, 0.0008, 0.0032),

        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.44, 0.33, 0.22)),
        
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_depenetration_velocity=1.0,
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=16,
        ),
        
        mass_props=sim_utils.MassPropertiesCfg(mass=0.5),
        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    ),
    init_state=AssetBaseCfg.InitialStateCfg(
        pos=(0.22, 0.25, 0.0),
        rot=(0.7071, 0.7071, 0.0, 0.0) 
    ),
)

@configclass
class BringupSceneCfg(InteractiveSceneCfg):
    ground     = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    dome_light = AssetBaseCfg(prim_path="/World/Light",  spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(1.0, 1.0, 1.0)))
    robot      = LITE6_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    
    aruco_marker_0 = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/ArucoMarker_0", spawn=sim_utils.UsdFileCfg(usd_path=usd_aruco_0, scale=(3.0, 3.0, 3.0), rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True)), init_state=AssetBaseCfg.InitialStateCfg(pos=(0.38, 0.13, 0.002)))
    aruco_marker_1 = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/ArucoMarker_1", spawn=sim_utils.UsdFileCfg(usd_path=usd_aruco_1, scale=(3.0, 3.0, 3.0), rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True)), init_state=AssetBaseCfg.InitialStateCfg(pos=(0.38, -0.13, 0.002)))
    box    = BOX_CFG.replace(prim_path="{ENV_REGEX_NS}/Box", init_state=AssetBaseCfg.InitialStateCfg(pos=(0.22, 0.25, 0.0), rot=(0.7071, 0.7071, 0.0, 0.0)))
    gear_0 = GEAR_CFG.replace(prim_path="{ENV_REGEX_NS}/Gear_0", init_state=AssetBaseCfg.InitialStateCfg(pos=(0.28,  0.05, 0.025)))
    gear_1 = GEAR_CFG.replace(prim_path="{ENV_REGEX_NS}/Gear_1", init_state=AssetBaseCfg.InitialStateCfg(pos=(0.38,  0.05, 0.025)))
    gear_2 = GEAR_CFG.replace(prim_path="{ENV_REGEX_NS}/Gear_2", init_state=AssetBaseCfg.InitialStateCfg(pos=(0.48,  0.05, 0.025)))
    gear_3 = GEAR_CFG.replace(prim_path="{ENV_REGEX_NS}/Gear_3", init_state=AssetBaseCfg.InitialStateCfg(pos=(0.28,  0.15, 0.025)))
    gear_4 = GEAR_CFG.replace(prim_path="{ENV_REGEX_NS}/Gear_4", init_state=AssetBaseCfg.InitialStateCfg(pos=(0.38,  0.15, 0.025)))
    gear_5 = GEAR_CFG.replace(prim_path="{ENV_REGEX_NS}/Gear_5", init_state=AssetBaseCfg.InitialStateCfg(pos=(0.48,  0.15, 0.025)))

    wrist_camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/UF_ROBOT/Geometry/world/link_base/link1/link2/link3/link4/link5/link6/link_eef/camera_link/Camera",
        update_period=0.0, height=480, width=840, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=1.93, horizontal_aperture=3.896, clipping_range=(0.01, 5.0)),
        offset=CameraCfg.OffsetCfg(pos=(0.003, 0.0, 0.0), rot=(-0.5, 0.5, -0.5, 0.5), convention="ros"),
    )

def setup_all_ros2_bridges(robot_prim_path: str, camera_render_product_path: str, ee_prim_path: str):
    mgr = omni.kit.app.get_app().get_extension_manager()

    for ext in ("isaacsim.ros2.bridge", "isaacsim.core.nodes", "omni.graph.action"):
        mgr.set_extension_enabled_immediate(ext, True)
    
    omni.kit.app.get_app().update()

    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/ROS2_Bridge", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick",                                     "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime",                                "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Context",                                    "isaacsim.ros2.bridge.ROS2Context"),
                    ("PublishClock",                               "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("PublishJointState",                          "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("SubscribeJointState",                        "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController",                     "isaacsim.core.nodes.IsaacArticulationController"),
                    ("CameraHelper",                               "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("PublishTF",                                  "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("PubEE",                                      "isaacsim.ros2.bridge.ROS2Publisher"),
                    ("JointTrajServer", "isaacsim.ros2.bridge.ROS2ControlFollowJointTrajectory"),
                ],

                keys.CONNECT: [
                    ("OnTick.outputs:tick",                        "PublishClock.inputs:execIn"),
                    ("OnTick.outputs:tick",                        "PublishJointState.inputs:execIn"),
                    ("OnTick.outputs:tick",                        "SubscribeJointState.inputs:execIn"),
                    ("OnTick.outputs:tick",                        "ArticulationController.inputs:execIn"),
                    ("OnTick.outputs:tick",                        "CameraHelper.inputs:execIn"),
                    ("OnTick.outputs:tick",                        "PublishTF.inputs:execIn"),
                    ("OnTick.outputs:tick",                        "PubEE.inputs:execIn"),

                    ("Context.outputs:context",                    "PublishClock.inputs:context"),
                    ("Context.outputs:context",                    "PublishJointState.inputs:context"),
                    ("Context.outputs:context",                    "SubscribeJointState.inputs:context"),
                    ("Context.outputs:context",                    "CameraHelper.inputs:context"),
                    ("Context.outputs:context",                    "PublishTF.inputs:context"),
                    ("Context.outputs:context",                    "PubEE.inputs:context"),

                    ("ReadSimTime.outputs:simulationTime",          "PublishClock.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime",          "PublishJointState.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime",          "PublishTF.inputs:timeStamp"),

                    ("SubscribeJointState.outputs:jointNames",      "ArticulationController.inputs:jointNames"),
                    ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),

                    ("OnTick.outputs:tick",         "JointTrajServer.inputs:execIn"),
                    ("Context.outputs:context",     "JointTrajServer.inputs:context"),
                    ("JointTrajServer.outputs:jointNames",    "ArticulationController.inputs:jointNames"),
                    ("JointTrajServer.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ],

                keys.SET_VALUES: [
                    ("PublishJointState.inputs:targetPrim",         [usdrt.Sdf.Path(robot_prim_path)]),
                    ("ArticulationController.inputs:robotPath",     robot_prim_path),
                    ("PublishJointState.inputs:topicName",          "joint_states"),
                    ("SubscribeJointState.inputs:topicName",        "joint_commands"),
                    ("CameraHelper.inputs:renderProductPath",       camera_render_product_path),
                    ("CameraHelper.inputs:topicName",               "wrist_camera/image_raw"),
                    ("PublishTF.inputs:targetPrims",                [usdrt.Sdf.Path(robot_prim_path)]),
                    ("PubEE.inputs:topicName",                      "ee_pose"),
                    ("PubEE.inputs:messagePackage",                 "geometry_msgs"),
                    ("PubEE.inputs:messageName",                    "Pose"),
                    ("JointTrajServer.inputs:actionNamespace", "joint_trajectory_controller"),
                    ("JointTrajServer.inputs:robotPath",       ARTIC_PRIM_PATH),
                ],
            },
        )
        print(f"[INFO] ROS 2 All-in-One Bridge Complete.")
    except Exception as e:
        print(f"[ERROR] Bridge Failed: {e}")

def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device="cpu")
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[2.0, 2.0, 2.0], target=[0.0, 0.0, 0.0])

    scene = InteractiveScene(BringupSceneCfg(num_envs=1, env_spacing=3.0))
    sim.reset()

    robot: Articulation = scene["robot"]
    robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
    robot.set_joint_position_target(robot.data.default_joint_pos)
    robot_cfg = SceneEntityCfg("robot", joint_names=["joint[1-6]"], body_names=["link6"])
    robot_cfg.resolve(scene)
    ee_jacobi_idx = robot_cfg.body_ids[0] - 1

    robot_prim_path = robot.root_physx_view.prim_paths[0]

    stage = omni.usd.get_context().get_stage()
    ee_prim_path = robot_prim_path
    for prim in stage.Traverse():
        if prim.GetName() == "link6" and robot_prim_path in str(prim.GetPath()):
            ee_prim_path = str(prim.GetPath())
            break

    import omni.replicator.core as rep
    camera_prim_path = robot_prim_path + "/link1/link2/link3/link4/link5/link6/link_eef/camera_link/Camera"
    
    render_product = rep.create.render_product(camera_prim_path, (840, 480))

    setup_all_ros2_bridges(robot_prim_path, render_product.path, ee_prim_path)

    print("\n==================================================================")
    print("[INFO] Isaac Sim Bring-up Node Running...")
    print("[INFO] TF is active. Check: ros2 run tf2_ros tf2_echo World link6")
    print("==================================================================\n", flush=True)

    p_x = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:position:x")
    p_y = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:position:y")
    p_z = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:position:z")
    q_w = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:orientation:w")
    q_x = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:orientation:x")
    q_y = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:orientation:y")
    q_z = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:orientation:z")

    while simulation_app.is_running():
        sim.step()
        scene.update(0.01)

        ee_pos_tensor = robot.data.body_pos_w[:, robot_cfg.body_ids[0], :]
        ee_pos_list = ee_pos_tensor[0].tolist()

        ee_quat_tensor = robot.data.body_quat_w[:, robot_cfg.body_ids[0], :]
        ee_quat_list = ee_quat_tensor[0].tolist() 

        try:
            # --- Position    (x, y, z)    ---
            if p_x: og.Controller.set(p_x, float(ee_pos_list[0]))
            if p_y: og.Controller.set(p_y, float(ee_pos_list[1]))
            if p_z: og.Controller.set(p_z, float(ee_pos_list[2]))

            # --- Quaternion  (w, x, y, z) ---      
            if q_w: og.Controller.set(q_w, float(ee_quat_list[0]))
            if q_x: og.Controller.set(q_x, float(ee_quat_list[1]))
            if q_y: og.Controller.set(q_y, float(ee_quat_list[2]))
            if q_z: og.Controller.set(q_z, float(ee_quat_list[3]))

        except Exception as e:
            pass

if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print("Bringup.py Exit.")

    finally:
        simulation_app.close()