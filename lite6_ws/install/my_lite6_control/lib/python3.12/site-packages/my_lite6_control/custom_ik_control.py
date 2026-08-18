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

limit_min = np.array([-6.28, -2.61, -0.06, -6.28, -2.16, -6.28])
limit_max = np.array([ 6.28,  2.61,  5.23,  6.28,  2.16,  6.28])

L_MIN = limit_min
L_MAX = limit_max

joints = [
 [[0,       0,  0.2435], [0,            0,            0]],
 [[0,       0,       0], [1.5708,       -1.5708, 3.1416]],
 [[0.2002,  0,       0], [-3.1416,      0,       1.5708]],
 [[0.087,  -0.22761, 0], [1.5708,       0,            0]],
 [[0,       0,       0], [1.5708,       0,            0]],
 [[0,       0.0625,  0], [-1.5708,      0,            0]]]

#############################
### Euler Rotation Matrix ###
###       Z @ Y @ X       ###
#############################
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

#########################################
### Homogeneous Transformation Matrix ###
###         ------------------        ###
###         | r11 r12 r13  x |        ###
###     T = | r21 r22 r23  y |        ###
###         | r31 r32 r33  z |        ###
###         |  0   0   0   1 |        ###
###         ------------------        ###
#########################################
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

#########################################
###     Quaternion Transform to R     ###
#########################################
def quat_to_R(w, x, y, z):
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])

#########################
### Foward Kinematics ###
###  T0 T1 ... T5 T6  ###
#########################
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
        super().__init__('IK_Calculate')

        self.sub = self.create_subscription(Pose, '/target_pose', self._cb_target_pose, 10)
        # self.joint_pub = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self.pub = self.create_publisher(JointTrajectory, '/sim_lite6_traj_controller/joint_trajectory', 10)
        # self.theta = np.array([0.0, 0.7, 1.57, 0.0, 0.87, 0.0])
        self.theta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def _cb_target_pose(self, msg: Pose):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        threshold = np.sqrt(x**2 + y**2 + z**2)

        target_pos = np.array([x, y, z])
        target_R   = quat_to_R(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

        lam = 0.1    # DLS Damping factor
        alpha = 0.5  # Learning rate
        
        theta = self.theta.copy()

        for step in range(2000):
            T, pos, quat, zs, ps = FK(theta)
            R_error = target_R @ T[:3, :3].T
            dP = target_pos - pos
            dW = np.array([R_error[2,1] - R_error[1,2],
                           R_error[0,2] - R_error[2,0],
                           R_error[1,0] - R_error[0,1]]) * 0.5
            dX = np.append(dP, dW)
            self.get_logger().info(f"{dW}")

            error_norm = np.linalg.norm(dX)
            
            if error_norm < 1e-3:
                self.get_logger().info(f"[Success] Converged at step {step}")
                break
                
            J = np.zeros((6, 6))
            for i in range(6):
                r = pos - ps[i]
                J[:3, i] = np.cross(zs[i], r)
                J[3:, i] = zs[i]
                
            manip  = np.sqrt(abs(np.linalg.det(J @ J.T)))
            lam    = 0.1 + 0.4 * np.exp(-manip / 0.05)
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(6))
            theta  = np.clip(theta + 0.5 * np.clip(J_pinv @ dX, -0.1, 0.1), L_MIN, L_MAX)
        else:
            T, pos, quat, _, _ = FK(theta)
            self.get_logger().warn(f"[FAIL step=500] final quat={np.round(quat,3)} err={np.linalg.norm(dX):.4f}")

        delta    = float(np.max(np.abs(theta - self.theta)))

        move_sec  = max(3.0, float(delta) * 16.0)  # motor speed parameter

        self.theta = theta
        pt = JointTrajectoryPoint()
        pt.positions = theta.tolist()
        pt.time_from_start = Duration(sec=int(move_sec), nanosec=int((move_sec % 1) * 1e9))
        jt = JointTrajectory()
        jt.header.stamp  = self.get_clock().now().to_msg()
        jt.joint_names   = ['joint1','joint2','joint3','joint4','joint5','joint6']
        jt.points        = [pt]
        self.pub.publish(jt)


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