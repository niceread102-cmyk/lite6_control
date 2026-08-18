from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints,
                              PositionConstraint, OrientationConstraint,
                              BoundingVolume)
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation, Slerp
import numpy as np

EEF_LINK    = 'link_eef'
BASE_FRAME  = 'link_base'
TCP_OFFSET  = 0.0836
NEUTRAL_Q   = (1.0, 0.0, 0.0, 0.0)  # Rx(180 deg), gripper pointing straight down


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


def build_move_goal(pose: Pose, vel_scale=0.5, acc_scale=0.3):
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
    goal_msg.planning_options.plan_only  = True
    return goal_msg
