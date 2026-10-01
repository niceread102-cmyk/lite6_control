"""MoveIt request builders and joint-limit handling shared by the config-driven nodes (moveit2,
moveit2_action, moveit2_test, sequence_sim_action, sequence_sim_action_fix, sequence_real_action).
Every number comes from the config (config/default.yaml, loaded by my_lite6_control/config.py)."""
import math

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from moveit_msgs.srv import GetPositionIK
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from my_lite6_control.config import joint_limits, limit_violations

NO_IK_SOLUTION = -31  # moveit_msgs/MoveItErrorCodes


def to_duration(seconds: float) -> Duration:
    """Duration from float seconds, keeping the fraction (Duration(sec=int(s)) drops it)."""
    ns = int(round(max(0.0, float(seconds)) * 1e9))
    return Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def to_seconds(d: Duration) -> float:
    return d.sec + d.nanosec * 1e-9


def eef_target_from_tcp(pose: Pose, cfg) -> PoseStamped:
    """TCP pose -> pose of the IK link (robot.eef_link): back off robot.tcp_offset_m along the tool z axis."""
    R       = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w]).as_matrix()
    tcp_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    eef_pos = tcp_pos - R @ np.array([0.0, 0.0, cfg.robot.tcp_offset_m])

    target = PoseStamped()
    target.header.frame_id = cfg.robot.base_frame
    target.pose.position.x, target.pose.position.y, target.pose.position.z = eef_pos.tolist()
    target.pose.orientation = pose.orientation
    return target


def joint_limit_constraints(cfg) -> list:
    """robot.joint_limits as JointConstraints (centre +- half range), for IK and path constraints."""
    out = []
    for lim in joint_limits(cfg):
        half = 0.5 * (lim.upper - lim.lower)
        out.append(JointConstraint(joint_name=lim.name, position=0.5 * (lim.upper + lim.lower),
                                   tolerance_above=half, tolerance_below=half, weight=1.0))
    return out


def build_ik_request(target: PoseStamped, joint_state: JointState, cfg) -> GetPositionIK.Request:
    # compute_ik seeded with the CURRENT joint state -> converges to the nearest
    # joint solution (Newton/Jacobian), instead of MoveGroup picking an arbitrary
    # branch inside a loose position/orientation tolerance region (the source of
    # the micro-jitter/wild-swing behaviour). With planning.enforce_joint_limits the
    # solution must also lie inside robot.joint_limits: MoveIt only retries from
    # other seeds when the nearest solution is outside them.
    req                                    = GetPositionIK.Request()
    req.ik_request.group_name              = cfg.robot.group
    req.ik_request.robot_state.joint_state = joint_state
    req.ik_request.avoid_collisions        = bool(cfg.planning.ik.avoid_collisions)
    req.ik_request.ik_link_name            = cfg.robot.eef_link
    req.ik_request.pose_stamped            = target
    req.ik_request.timeout                 = to_duration(cfg.planning.ik.timeout_s)
    if cfg.planning.enforce_joint_limits:
        req.ik_request.constraints = Constraints(name='joint_limits', joint_constraints=joint_limit_constraints(cfg))
    return req


def ik_joint_targets(response, cfg):
    """(joint values in robot.joint_names order, None) from a GetPositionIK response, or (None, reason)."""
    code = response.error_code.val
    if code != 1:
        hint = ''
        if code == NO_IK_SOLUTION and cfg.planning.enforce_joint_limits:
            hint = ' (no solution inside robot.joint_limits, or the pose is unreachable)'
        return None, f'compute_ik error_code={code}{hint}'

    solved = dict(zip(response.solution.joint_state.name,
                      response.solution.joint_state.position))
    try:
        joint_targets = [solved[n] for n in cfg.robot.joint_names]
    except KeyError:
        return None, 'IK solution missing expected joints'

    if cfg.planning.enforce_joint_limits:
        bad = limit_violations(cfg, joint_targets)
        if bad:
            return None, 'IK solution outside robot.joint_limits: ' + '; '.join(bad)
    return joint_targets, None


def build_joint_goal(joint_targets, cfg, vel_scale=None, acc_scale=None, plan_only=True) -> MoveGroup.Goal:
    # JointConstraint tolerance (planning.goal_joint_tolerance_rad): unlike the SO(3)
    # rejection sampling of Position/OrientationConstraints, a joint tolerance stays
    # easy to sample however tight it is - it shrinks OMPL's goal region to practically
    # one point, so the plan lands on the same spot every time (the root cause of the
    # micro-jitter / random branch jumps).
    p   = cfg.planning
    tol = float(p.goal_joint_tolerance_rad)
    joint_con = [JointConstraint(joint_name=n, position=float(q),
                                 tolerance_above=tol, tolerance_below=tol, weight=1.0)
                 for n, q in zip(cfg.robot.joint_names, joint_targets)]

    ws_x, ws_y, ws_z = cfg.safety.workspace_x_m, cfg.safety.workspace_y_m, cfg.safety.workspace_z_m
    req                                       = MotionPlanRequest()
    req.group_name                            = cfg.robot.group
    req.pipeline_id                           = p.get('pipeline_id') or ''
    req.planner_id                            = p.planner_id
    req.start_state.is_diff                   = True
    req.workspace_parameters.header.frame_id  = cfg.robot.base_frame
    req.workspace_parameters.min_corner.x     = float(ws_x[0])
    req.workspace_parameters.min_corner.y     = float(ws_y[0])
    req.workspace_parameters.min_corner.z     = float(ws_z[0])
    req.workspace_parameters.max_corner.x     = float(ws_x[1])
    req.workspace_parameters.max_corner.y     = float(ws_y[1])
    req.workspace_parameters.max_corner.z     = float(ws_z[1])
    req.goal_constraints                      = [Constraints(joint_constraints=joint_con)]
    if p.enforce_joint_limits:
        # every state of the planned path stays inside robot.joint_limits
        req.path_constraints = Constraints(name='joint_limits', joint_constraints=joint_limit_constraints(cfg))
    req.num_planning_attempts                 = int(p.planning_attempts)
    req.allowed_planning_time                 = float(p.planning_time_s)
    req.max_velocity_scaling_factor           = float(p.velocity_scaling if vel_scale is None else vel_scale)
    req.max_acceleration_scaling_factor       = float(p.acceleration_scaling if acc_scale is None else acc_scale)

    goal_msg                             = MoveGroup.Goal()
    goal_msg.request                     = req
    goal_msg.planning_options.plan_only  = plan_only
    return goal_msg


def retime_to_limits(traj: JointTrajectory, cfg) -> float:
    """Slow a time-parameterized trajectory down uniformly (in place) until no joint exceeds its
    robot.joint_limits max velocity / acceleration. Returns the time scale factor (1.0 = unchanged)."""
    limits = {lim.name: lim for lim in joint_limits(cfg)}
    cols = [(i, limits[n]) for i, n in enumerate(traj.joint_names)
            if n in limits and (limits[n].max_velocity or limits[n].max_acceleration)]
    if not cols or not traj.points:
        return 1.0

    scale = 1.0
    prev_t, prev = 0.0, None
    for pt in traj.points:
        t = to_seconds(pt.time_from_start)
        dt = t - prev_t if prev is not None else 0.0
        for i, lim in cols:
            if lim.max_velocity:
                if i < len(pt.velocities):
                    scale = max(scale, abs(pt.velocities[i]) / lim.max_velocity)
                if dt > 0:  # mean speed over the segment
                    scale = max(scale, abs(pt.positions[i] - prev.positions[i]) / dt / lim.max_velocity)
            if lim.max_acceleration:
                if i < len(pt.accelerations):
                    scale = max(scale, math.sqrt(abs(pt.accelerations[i]) / lim.max_acceleration))
                if dt > 0 and i < len(pt.velocities) and i < len(prev.velocities):
                    acc = abs(pt.velocities[i] - prev.velocities[i]) / dt
                    scale = max(scale, math.sqrt(acc / lim.max_acceleration))
        prev_t, prev = t, pt

    if scale <= 1.0 + 1e-9:
        return 1.0
    for pt in traj.points:
        pt.time_from_start = to_duration(to_seconds(pt.time_from_start) * scale)
        pt.velocities      = [v / scale for v in pt.velocities]
        pt.accelerations   = [a / (scale * scale) for a in pt.accelerations]
    return scale


def prepare_trajectory(traj: JointTrajectory, cfg, logger) -> bool:
    """robot.joint_limits applied to a planned trajectory before it is executed: refused if a waypoint is
    outside the position limits, slowed down (in place) to the velocity/acceleration limits.
    Returns False when it must not be executed. No-op with planning.enforce_joint_limits: false."""
    if not cfg.planning.enforce_joint_limits:
        return True
    names = list(traj.joint_names)
    for k, pt in enumerate(traj.points):
        bad = limit_violations(cfg, pt.positions, names)
        if bad:
            logger.error(f'[LIMIT] planned waypoint {k + 1}/{len(traj.points)} outside robot.joint_limits: '
                         + '; '.join(bad))
            return False
    scale = retime_to_limits(traj, cfg)
    if scale > 1.0:
        logger.info(f'[LIMIT] trajectory slowed down x{scale:.2f} to the joint velocity/acceleration limits '
                    f'(now {to_seconds(traj.points[-1].time_from_start):.2f} s)')
    return True


def fixed_move_duration(target, joint_state, cfg, requested_s: float) -> float:
    """requested_s, lengthened so no joint exceeds its robot.joint_limits max velocity (the controller
    interpolates a single-point trajectory linearly: peak speed = distance / duration)."""
    if not cfg.planning.enforce_joint_limits or joint_state is None:
        return requested_s
    current = dict(zip(joint_state.name, joint_state.position))
    need = 0.0
    for lim, q in zip(joint_limits(cfg), target):
        if lim.max_velocity and lim.name in current:
            need = max(need, abs(q - current[lim.name]) / lim.max_velocity)
    return max(requested_s, need)


def joint_trajectory(cfg, positions, duration_s: float, stamp) -> JointTrajectory:
    """Single-point trajectory to `positions` (robot.joint_names order), reached after duration_s."""
    pt = JointTrajectoryPoint()
    pt.positions       = [float(q) for q in positions]
    pt.time_from_start = to_duration(duration_s)
    jt = JointTrajectory()
    jt.header.stamp = stamp
    jt.joint_names  = list(cfg.robot.joint_names)
    jt.points       = [pt]
    return jt


def pose_is_safe(msg: Pose, logger, cfg) -> bool:
    """/target_pose check against the safety box and reach (config `safety`)."""
    ws_x = tuple(cfg.safety.workspace_x_m)
    ws_y = tuple(cfg.safety.workspace_y_m)
    ws_z = tuple(cfg.safety.workspace_z_m)
    max_reach = cfg.safety.max_reach_m
    p = np.array([msg.position.x, msg.position.y, msg.position.z])

    if not np.all(np.isfinite(p)):
        logger.error(f'target has NaN/Inf, DENIED: {p}')
        return False

    if not (ws_x[0] <= p[0] <= ws_x[1] and
            ws_y[0] <= p[1] <= ws_y[1] and
            ws_z[0] <= p[2] <= ws_z[1]):
        logger.error(f'target {np.round(p, 3)} outside workspace bounds '
                     f'x{ws_x} y{ws_y} z{ws_z}, DENIED')
        return False

    reach = float(np.linalg.norm(p))
    if reach > max_reach:
        logger.error(f'target {np.round(p, 3)} reach={reach:.3f}m > {max_reach}m, DENIED')
        return False

    return True


def pre_grasp_pose(msg: Pose, dist: float) -> Pose:
    """msg backed off by dist along its tool z axis (negative dist = further along it)."""
    R = Rotation.from_quat([msg.orientation.x, msg.orientation.y,
                             msg.orientation.z, msg.orientation.w]).as_matrix()
    approach_dir = R[:, 2]
    pos = np.array([msg.position.x, msg.position.y, msg.position.z]) - dist * approach_dir

    pre = Pose()
    pre.position.x, pre.position.y, pre.position.z = pos.tolist()
    pre.orientation = msg.orientation
    return pre


def poses_close(a: Pose, b: Pose, pos_eps_m: float, angle_eps_deg: float) -> bool:
    pa = np.array([a.position.x, a.position.y, a.position.z])
    pb = np.array([b.position.x, b.position.y, b.position.z])
    if np.linalg.norm(pa - pb) > pos_eps_m:
        return False
    qa = np.array([a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w])
    qb = np.array([b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w])
    dot = np.clip(abs(np.dot(qa, qb)), 0.0, 1.0)
    angle_deg = np.degrees(2 * np.arccos(dot))
    return angle_deg <= angle_eps_deg
