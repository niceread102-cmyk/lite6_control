import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

np.set_printoptions(precision=3, suppress=True)

# lite6 urdf joint and rpy information.
# target xyz pos 0.1 = 10cm

joints = [
 [[0,       0,  0.2435], [0,            0,            0]],
 [[0,       0,       0], [1.5708,       -1.5708, 3.1416]],
 [[0.2002,  0,       0], [-3.1416,      0,       1.5708]],
 [[0.087,  -0.22761, 0], [1.5708,       0,            0]],
 [[0,       0,       0], [1.5708,       0,            0]],
 [[0,       0.0625,  0], [-1.5708,      0,            0]]]

def euler_rotation_matrix(x, y, z):
    Rx = np.array([[1,         0,          0],
                   [0, np.cos(x), -np.sin(x)],
                   [0, np.sin(x),  np.cos(x)]])

    Ry = np.array([[ np.cos(y), 0, np.sin(y)],
                   [ 0,         1,         0],
                   [-np.sin(y), 0, np.cos(y)]])

    Rz = np.array([[np.cos(z), -np.sin(z), 0],
                   [np.sin(z),  np.cos(z), 0],
                   [0,         0,          1]])
        
    return Rz @ Ry @ Rx

def make_transform(xyz, rpy):
    R = euler_rotation_matrix(*rpy)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = xyz
    return T

def rotation_to_quaternion(R):
    trace = R[0,0] + R[1,1] + R[2,2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s

    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s

    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s

    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s

    return np.array([w, x, y, z])

def quaternion_to_rotation_matrix(x, y, z, w):
    return np.array([
        [1 - 2*(y**2 + z**2),   2*(x*y - w*z),         2*(x*z + w*y)],
        [2*(x*y + w*z),         1 - 2*(x**2 + z**2),   2*(y*z - w*x)],
        [2*(x*z - w*y),         2*(y*z + w*x),         1 - 2*(x**2 + y**2)]])

def FK(theta):
    z_axes_local = []
    positions_local = []
    
    T_total = np.eye(4)
    for i, joint in enumerate(joints):
        T_total = T_total @ make_transform(joint[0], joint[1])
        z_axes_local.append(T_total[:3, 2])     # z_i
        positions_local.append(T_total[:3, 3])  # p_i
        T_total = T_total @ make_transform([0,0,0], [0, 0, theta[i]])

    pos  = T_total[:3, 3]
    quat = rotation_to_quaternion(T_total[:3, :3])
    
    return T_total, pos, quat, z_axes_local, positions_local


class IK_Calculate(Node):
    def __init__(self):
        super().__init__('IK_calculate')

        self.target_sub = self.create_subscription(Pose, '/target_pose', self._cb_target_pose, 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        
        self.theta = np.array([0.0, 0.7, 1.57, 0.0, 0.87, 0.0])

    def _cb_target_pose(self, msg: Pose):
        px, py, pz = msg.position.x, msg.position.y, msg.position.z
        threshold = np.sqrt(px ** 2 + py ** 2 + pz ** 2)

        if 0.5 >= threshold and pz <= 0.05:
            self.get_logger().warn(f"denied: pos=[{px:.3f},{py:.3f},{pz:.3f}]")
            return

        target_pos = np.array([px, py, pz])
        target_R   = quaternion_to_rotation_matrix(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

        lam = 0.1    # DLS Damping factor
        alpha = 0.5  # Learning rate
        
        theta_current = self.theta.copy()
        
        converged = False
        pos = np.zeros(3)

        for step in range(200):
            T, pos, quat, cur_z_axes, cur_positions = FK(theta_current)

            self.get_logger().info(f"quaternion {quat}")
            
            R_error = target_R @ T[:3, :3].T
            dP = target_pos - pos
            dW = np.array([R_error[2,1] - R_error[1,2],
                           R_error[0,2] - R_error[2,0],
                           R_error[1,0] - R_error[0,1]]) * 0.5
            dX = np.append(dP, dW)
            
            error_norm = np.linalg.norm(dX)
            
            if error_norm < 1e-3:
                self.get_logger().info(f"[Success] Converged at step {step}")
                converged = True
                break
                
            J = np.zeros((6, 6))
            for i in range(6):
                r = pos - cur_positions[i]
                J[:3, i] = np.cross(cur_z_axes[i], r)
                J[3:, i] = cur_z_axes[i]
                
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(6))

            d_theta = np.clip(J_pinv @ dX, -0.1, 0.1)
            theta_current = theta_current + alpha * d_theta

            theta_current = np.arctan2(np.sin(theta_current), np.cos(theta_current))

            limit_min = np.array([-6.28, -2.61, -0.06, -6.28, -2.16, -6.28])
            limit_max = np.array([ 6.28,  2.61,  5.23,  6.28,  2.16,  6.28])
            
            theta_current = np.clip(theta_current, limit_min, limit_max)

        if converged:
            delta = np.max(np.abs(theta_current - self.theta))
            if delta > 1.5:
                self.get_logger().warn(f"IK jump too large: {delta:.3f} rad, skipping")
                return

            move_sec  = max(3.0, float(delta) * 8.0)  # motor speed parameter
            self.theta = theta_current

            pt = JointTrajectoryPoint()
            pt.positions       = self.theta.tolist()
            pt.time_from_start = Duration(sec=int(move_sec), nanosec=int((move_sec % 1) * 1e9))
            joint_msg = JointTrajectory()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.joint_names  = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            joint_msg.points       = [pt]

            self.joint_pub.publish(joint_msg)
            self.get_logger().info(f"Target: [{px:.3f}, {py:.3f}, {pz:.3f}] -> Final EE Pos: {pos}")
        else:
            self.get_logger().warn(f"IK Failed to converge for Target: [{px:.3f}, {py:.3f}, {pz:.3f}]")

def main(args=None):
    rclpy.init(args=args)
    node = IK_Calculate()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()