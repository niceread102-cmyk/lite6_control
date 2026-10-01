from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import (MotionPlanRequest, Constraints,
                              PositionConstraint, OrientationConstraint,
                              BoundingVolume, JointConstraint)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation, Slerp
import numpy as np

EEF_LINK    = 'link_eef'
BASE_FRAME  = 'link_base'
TCP_OFFSET  = 0.0836
NEUTRAL_Q   = (1.0, 0.0, 0.0, 0.0)  # Rx(180 deg), gripper pointing straight down
ARM_JOINTS  = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')

# Must match sequence_sim_action.py's WS_X/WS_Y/WS_Z — this is the actual
# planning volume OMPL samples in; without it move_group falls back to its
# own default box, which doesn't match the robot's real reachable space.
WS_X = (0.05, 0.45)
WS_Y = (-0.45, 0.45)
WS_Z = (0.01, 0.60)


def build_approach_waypoints(target_pose: Pose, dist=0.1, steps=5):
    """Interpolate from a neutral, retreated approach point down to the
    fully-tilted target pose. As distance-to-go shrinks, tilt increases."""
    target_pos = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])

    R_neutral = Rotation.from_quat(NEUTRAL_Q)
    R_target  = Rotation.from_quat([target_pose.orientation.x, target_pose.orientation.y,
                                     target_pose.orientation.z, target_pose.orientation.w])
    slerp = Slerp([0, 1], Rotation.from_quat([R_neutral.as_quat(), R_target.as_quat()]))

    waypoints = []
    for i in range(steps + 1):
        t   = i / steps
        R_t = slerp([t])[0]
        approach_dir = R_t.as_matrix()[:, 2]
        pos_t = target_pos + (1.0 - t) * dist * approach_dir

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos_t.tolist()
        q = R_t.as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        waypoints.append(pose)

    return waypoints


def build_move_goal(pose: Pose, vel_scale=0.5, acc_scale=0.3, plan_only=True,
                     pipeline_id='pilz_industrial_motion_planner', planner_id='PTP'):
    R       = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w]).as_matrix()
    tcp_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    eef_pos = tcp_pos - R @ np.array([0.0, 0.0, TCP_OFFSET])

    target = PoseStamped()
    target.header.frame_id = BASE_FRAME
    target.pose.position.x, target.pose.position.y, target.pose.position.z = eef_pos.tolist()
    target.pose.orientation = pose.orientation

    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])
    bv     = BoundingVolume(primitives=[sphere], primitive_poses=[target.pose])

    pos_con                   = PositionConstraint()
    pos_con.header            = target.header
    pos_con.link_name         = EEF_LINK
    pos_con.constraint_region = bv
    pos_con.weight            = 1.0

    ori_con                           = OrientationConstraint()
    ori_con.header                    = target.header
    ori_con.link_name                 = EEF_LINK
    ori_con.orientation               = target.pose.orientation
    ori_con.absolute_x_axis_tolerance = 0.05
    ori_con.absolute_y_axis_tolerance = 0.05
    ori_con.absolute_z_axis_tolerance = 0.05
    ori_con.weight                    = 1.0

    req                                 = MotionPlanRequest()
    req.group_name                      = 'lite6'
    req.pipeline_id                     = pipeline_id
    req.planner_id                      = planner_id
    req.start_state.is_diff             = True
    req.workspace_parameters.header.frame_id = BASE_FRAME
    req.workspace_parameters.min_corner.x    = WS_X[0]
    req.workspace_parameters.min_corner.y    = WS_Y[0]
    req.workspace_parameters.min_corner.z    = WS_Z[0]
    req.workspace_parameters.max_corner.x    = WS_X[1]
    req.workspace_parameters.max_corner.y    = WS_Y[1]
    req.workspace_parameters.max_corner.z    = WS_Z[1]
    req.goal_constraints                = [Constraints(
        position_constraints    = [pos_con],
        orientation_constraints = [ori_con],
    )]
    req.num_planning_attempts           = 10
    req.allowed_planning_time           = 5.0
    req.max_velocity_scaling_factor     = vel_scale
    req.max_acceleration_scaling_factor = acc_scale

    goal_msg                             = MoveGroup.Goal()
    goal_msg.request                     = req
    goal_msg.planning_options.plan_only  = plan_only
    return goal_msg


def build_ik_request(pose: Pose, group_name='lite6', ik_link_name=EEF_LINK, timeout_sec=2.0):
    """IK request seeded with the current robot state (is_diff=True), so the
    solver returns the joint solution closest to where the arm already is
    instead of an arbitrary branch (e.g. a 180 deg wrist/elbow flip)."""
    R       = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w]).as_matrix()
    tcp_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    eef_pos = tcp_pos - R @ np.array([0.0, 0.0, TCP_OFFSET])

    ps = PoseStamped()
    ps.header.frame_id = BASE_FRAME
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = eef_pos.tolist()
    ps.pose.orientation = pose.orientation

    req = GetPositionIK.Request()
    req.ik_request.group_name          = group_name
    req.ik_request.ik_link_name        = ik_link_name
    req.ik_request.pose_stamped        = ps
    req.ik_request.robot_state.is_diff = True
    req.ik_request.avoid_collisions    = True
    req.ik_request.timeout             = Duration(sec=int(timeout_sec))
    return req


def build_joint_goal(joint_state: JointState, group_name='lite6', vel_scale=0.5, acc_scale=0.3):
    constraints = Constraints()
    for name, pos in zip(joint_state.name, joint_state.position):
        if name not in ARM_JOINTS:
            continue
        constraints.joint_constraints.append(JointConstraint(
            joint_name      = name,
            position        = pos,
            tolerance_above = 0.001,
            tolerance_below = 0.001,
            weight          = 1.0,
        ))

    req                                  = MotionPlanRequest()
    req.group_name                       = group_name
    req.goal_constraints                 = [constraints]
    req.num_planning_attempts            = 10
    req.allowed_planning_time            = 5.0
    req.max_velocity_scaling_factor      = vel_scale
    req.max_acceleration_scaling_factor  = acc_scale

    goal_msg                             = MoveGroup.Goal()
    goal_msg.request                     = req
    goal_msg.planning_options.plan_only  = True
    return goal_msg
