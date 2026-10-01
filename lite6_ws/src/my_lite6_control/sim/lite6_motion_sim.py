"""Lightweight Isaac Sim stage for watching Lite6 motion: a floor, two lights and the robot - no bin map, no
cameras, no image topics. The robot mirrors /joint_states (MoveIt fake controller or the real robot's driver),
so it moves exactly like the trajectories the robot logic executes; the gripper follows /gripper/command.
Settings: the `sim:` section of the config (joint limits and named poses: config/robots/lite6.yaml).

    lab sim/lite6_motion_sim.py                                         # config/default.yaml
    lab sim/lite6_motion_sim.py sim.render.mode=RealTimePathTracing sim.initial_pose=init
    lab sim/lite6_motion_sim.py --config ~/my.yaml --headless

Typical session (one terminal each):
    ros2 launch xarm_moveit_config lite6_moveit_fake_custom.launch.py add_gripper:=true
    lab ~/lite6_ws/src/my_lite6_control/sim/lite6_motion_sim.py
    ros2 run my_lite6_control sequence_sim_action_fix         (or: ros2 run my_lite6_control moveit2 + ee_6d.py)
"""
import argparse
import math
import os
import sys
import threading
import time

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # src/my_lite6_control
sys.path.insert(0, PKG_ROOT)

from my_lite6_control.config import describe, joint_limits, load_config, named_pose  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--config", default=os.path.join(PKG_ROOT, "config", "default.yaml"))
parser.add_argument("overrides", nargs="*", help="dotted config overrides, e.g. sim.physics_hz=240")

from isaaclab.app import AppLauncher  # noqa: E402

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
cfg = load_config(args.config, args.overrides)
S = cfg.sim
args.experience = S.experience
args.headless = bool(args.headless or S.headless)
if not args.headless:
    args.visualizer = ["kit"]
# renderer + loop rate as Kit start-up settings, so the heavier default renderer is never initialised
FPS_LIMIT = int(round(float(S.render.fps_limit or 0)))
kit_settings = [f"--/app/runLoops/main/rateLimitEnabled={'true' if FPS_LIMIT > 0 else 'false'}"]
if FPS_LIMIT > 0:
    kit_settings.append(f"--/app/runLoops/main/rateLimitFrequency={FPS_LIMIT}")
if S.render.mode:
    kit_settings.append(f"--/rtx/rendermode={S.render.mode}")
args.kit_args = " ".join([getattr(args, "kit_args", "") or ""] + kit_settings).strip()
simulation_app = AppLauncher(args).app

import carb  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import rclpy  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float32  # noqa: E402

JOINTS  = list(cfg.robot.joint_names)
LIMITS  = joint_limits(cfg)
GRIPPER = cfg.robot.gripper
STALE_S = 0.2  # /joint_states older than this -> no velocity feed-forward (hold the last position)


class JointStateMirror(Node):
    """Latest arm joint positions/velocities from /joint_states and the gripper command, for the render loop."""

    def __init__(self, initial):
        super().__init__("lite6_motion_sim")
        self._lock    = threading.Lock()
        self._index   = {n: i for i, n in enumerate(JOINTS)}
        self._pos     = list(initial)
        self._vel     = [0.0] * len(JOINTS)
        self._gripper = float(GRIPPER.open_m)
        self._stamp   = None  # monotonic time of the last /joint_states that had arm joints
        self._count   = 0
        self.create_subscription(JointState, cfg.ros.joint_states_topic, self._cb_joint_state, 10)
        self.create_subscription(Float32, cfg.ros.gripper_command_topic, self._cb_gripper, 10)

    def _cb_joint_state(self, msg: JointState):
        with self._lock:
            hit = False
            for k, name in enumerate(msg.name):
                i = self._index.get(name)
                if i is None or k >= len(msg.position) or not math.isfinite(msg.position[k]):
                    continue
                self._pos[i] = float(msg.position[k])
                v = float(msg.velocity[k]) if k < len(msg.velocity) else 0.0
                self._vel[i] = v if math.isfinite(v) else 0.0
                hit = True
            if hit:
                self._stamp = time.monotonic()
                self._count += 1

    def _cb_gripper(self, msg: Float32):
        lo, hi = sorted((float(GRIPPER.close_m), float(GRIPPER.open_m)))
        with self._lock:
            self._gripper = min(max(float(msg.data), lo), hi)

    def snapshot(self):
        with self._lock:
            return list(self._pos), list(self._vel), self._gripper, self._stamp, self._count


def update(n=1):
    for _ in range(n):
        simulation_app.update()


def add_physics_scene(stage):
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx.CreateTimeStepsPerSecondAttr(int(S.physics_hz))
    physx.CreateEnableGPUDynamicsAttr(bool(S.physics_gpu))
    physx.CreateBroadphaseTypeAttr("GPU" if S.physics_gpu else "MBP")


def add_ground(stage, half=1.5, z=-0.001):
    """Plain grey quad: no material, no collider."""
    ground = UsdGeom.Mesh.Define(stage, "/World/ground")
    ground.CreatePointsAttr([Gf.Vec3f(-half, -half, z), Gf.Vec3f(half, -half, z),
                             Gf.Vec3f(half, half, z), Gf.Vec3f(-half, half, z)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateNormalsAttr([Gf.Vec3f(0.0, 0.0, 1.0)] * 4)
    ground.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    ground.CreateExtentAttr([Gf.Vec3f(-half, -half, z), Gf.Vec3f(half, half, z)])
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.32, 0.33, 0.35)])


def add_lights(stage):
    if S.lights.dome_intensity > 0:
        UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(float(S.lights.dome_intensity))
    if S.lights.distant_intensity > 0:
        sun = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
        sun.CreateIntensityAttr(float(S.lights.distant_intensity))
        sun.CreateAngleAttr(1.0)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 40.0))


def build_stage():
    """sim.map_usd, or a new stage with only a floor and two lights; plus a physics scene if it has none."""
    ctx = omni.usd.get_context()
    if S.map_usd:
        if not ctx.open_stage(S.map_usd):
            raise RuntimeError(f"could not open sim.map_usd: {S.map_usd}")
    else:
        ctx.new_stage()
    update(3)
    stage = ctx.get_stage()
    if not S.map_usd:
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
        stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
        if S.ground:
            add_ground(stage)
        add_lights(stage)
    if not any(prim.IsA(UsdPhysics.Scene) for prim in stage.Traverse()):
        add_physics_scene(stage)
    return stage


def find_joints(root, names, joint_type):
    found = {}
    for prim in Usd.PrimRange(root):
        if prim.GetName() in names and prim.IsA(joint_type):
            found.setdefault(prim.GetName(), prim)
    return found


def add_robot(stage, initial):
    """Reference the robot USD; joint limits (robot.joint_limits), PD drives and the initial pose on its joints."""
    parent = os.path.dirname(S.robot_prim)
    if parent not in ("", "/") and not stage.GetPrimAtPath(parent).IsValid():
        UsdGeom.Xform.Define(stage, parent)
    robot = stage.DefinePrim(S.robot_prim, "Xform")
    robot.GetReferences().AddReference(S.robot_usd)

    arm = find_joints(robot, set(JOINTS), UsdPhysics.RevoluteJoint)
    missing = [n for n in JOINTS if n not in arm]
    if missing:
        raise RuntimeError(f"joints {missing} not found under {S.robot_prim} ({S.robot_usd})")
    arm_drives = []
    for name, lim, q0 in zip(JOINTS, LIMITS, initial):
        prim = arm[name]
        if S.apply_joint_limits:
            joint = UsdPhysics.RevoluteJoint(prim)
            joint.GetLowerLimitAttr().Set(math.degrees(lim.lower))
            joint.GetUpperLimitAttr().Set(math.degrees(lim.upper))
            if lim.max_velocity:
                PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr().Set(math.degrees(lim.max_velocity))
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(float(S.drive.stiffness))
        drive.CreateDampingAttr().Set(float(S.drive.damping))
        drive.CreateMaxForceAttr().Set(float(S.drive.max_force))
        drive.CreateTargetPositionAttr().Set(math.degrees(q0))
        drive.CreateTargetVelocityAttr().Set(0.0)
        PhysxSchema.JointStateAPI.Apply(prim, "angular").CreatePositionAttr().Set(math.degrees(q0))
        arm_drives.append(drive)

    fingers = find_joints(robot, set(GRIPPER.joint_names), UsdPhysics.PrismaticJoint)
    if len(fingers) < len(GRIPPER.joint_names):
        print(f"[WARN] gripper joints {sorted(set(GRIPPER.joint_names) - set(fingers))} not found - "
              f"the gripper will not move", flush=True)
    gripper_drives = []
    for name in GRIPPER.joint_names:
        if name not in fingers:
            continue
        drive = UsdPhysics.DriveAPI.Apply(fingers[name], "linear")
        drive.CreateStiffnessAttr().Set(float(S.gripper_drive.stiffness))
        drive.CreateDampingAttr().Set(float(S.gripper_drive.damping))
        drive.CreateMaxForceAttr().Set(float(S.gripper_drive.max_force))
        drive.CreateTargetPositionAttr().Set(float(GRIPPER.open_m))
        gripper_drives.append(drive)

    # Nothing in this stage needs robot self-collision: it only costs solver time and can push links apart.
    for prim in Usd.PrimRange(robot):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr().Set(False)
    return arm_drives, gripper_drives


def apply_extra_render_settings():
    settings = carb.settings.get_settings()
    for key, value in (S.render.extra_settings or {}).items():
        settings.set(str(key), value)


def set_viewport_camera():
    try:
        from omni.kit.viewport.utility import get_active_viewport
        from omni.kit.viewport.utility.camera_state import ViewportCameraState
        viewport = get_active_viewport()
        if viewport is None:  # headless
            return
        state = ViewportCameraState("/OmniverseKit_Persp", viewport)
        state.set_position_world(Gf.Vec3d(*map(float, S.camera.eye_m)), True)
        state.set_target_world(Gf.Vec3d(*map(float, S.camera.target_m)), True)
    except Exception as e:  # the viewport API moved in this Kit version
        print(f"[WARN] viewport camera not set: {e}", flush=True)


def main():
    initial = named_pose(cfg, S.initial_pose)
    node = None
    try:
        stage = build_stage()
        arm_drives, gripper_drives = add_robot(stage, initial)
        update(2)
        apply_extra_render_settings()
        set_viewport_camera()

        rclpy.init()
        node = JointStateMirror(initial)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        threading.Thread(target=executor.spin, daemon=True).start()

        omni.timeline.get_timeline_interface().play()
        print("\n" + "=" * 78)
        print(f"[READY] lite6 motion sim - {describe(cfg)}")
        print(f"        render {S.render.mode or 'Kit default'} @ {FPS_LIMIT or 'uncapped'} fps, "
              f"physics {S.physics_hz} Hz {'GPU' if S.physics_gpu else 'CPU'}, "
              f"map {S.map_usd or 'minimal (floor + lights)'}")
        print(f"        following {cfg.ros.joint_states_topic} (arm) and {cfg.ros.gripper_command_topic} (gripper), "
              f"joint limits {'applied' if S.apply_joint_limits else 'from the USD'}")
        print("=" * 78 + "\n", flush=True)

        n = len(JOINTS)
        last_pos, last_vel, last_grip = [None] * n, [None] * n, None
        warned = set()
        frames, t_log, count_log = 0, time.monotonic(), 0
        while simulation_app.is_running():
            simulation_app.update()

            pos, vel, grip, stamp, count = node.snapshot()
            now = time.monotonic()
            fresh = stamp is not None and now - stamp < STALE_S
            for i, (drive, lim) in enumerate(zip(arm_drives, LIMITS)):
                q, v = pos[i], (vel[i] if fresh and S.drive.velocity_feedforward else 0.0)
                if S.apply_joint_limits and not lim.lower <= q <= lim.upper:
                    # warn only for a real violation, not for float noise right at a limit
                    if JOINTS[i] not in warned and not lim.lower - 1e-4 <= q <= lim.upper + 1e-4:
                        warned.add(JOINTS[i])
                        print(f"[WARN] {JOINTS[i]}={math.degrees(q):+.1f} deg from {cfg.ros.joint_states_topic} is "
                              f"outside robot.joint_limits [{math.degrees(lim.lower):+.1f}, "
                              f"{math.degrees(lim.upper):+.1f}] deg - clamped (warned once per joint)", flush=True)
                    q, v = min(max(q, lim.lower), lim.upper), 0.0
                # only write changed targets: an idle robot costs no USD/PhysX updates
                deg, deg_s = math.degrees(q), math.degrees(v)
                if deg != last_pos[i]:
                    drive.GetTargetPositionAttr().Set(deg)
                    last_pos[i] = deg
                if deg_s != last_vel[i]:
                    drive.GetTargetVelocityAttr().Set(deg_s)
                    last_vel[i] = deg_s
            if grip != last_grip:
                for drive in gripper_drives:
                    drive.GetTargetPositionAttr().Set(grip)
                last_grip = grip

            frames += 1
            if S.log_every_s and now - t_log >= S.log_every_s:
                rate = (count - count_log) / (now - t_log)
                state = (f"{rate:.0f} Hz, last {now - stamp:.2f} s ago" if stamp is not None
                         else "nothing yet (MoveIt fake / real driver running?)")
                print(f"[sim] {frames / (now - t_log):5.1f} fps | {cfg.ros.joint_states_topic}: {state}", flush=True)
                frames, t_log, count_log = 0, now, count
    except KeyboardInterrupt:
        pass
    finally:
        print("[INFO] shutting down - releasing the ROS node and the Kit app", flush=True)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
