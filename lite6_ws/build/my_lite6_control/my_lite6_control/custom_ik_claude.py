import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

np.set_printoptions(precision=3, suppress=True)

joints = [
 [[0,       0,  0.2435], [0,        0,       0      ]],
 [[0,       0,       0], [1.5708,  -1.5708,  3.1416 ]],
 [[0.2002,  0,       0], [-3.1416,  0,       1.5708 ]],
 [[0.087,  -0.22761, 0], [1.5708,   0,       0      ]],
 [[0,       0,       0], [1.5708,   0,       0      ]],
 [[0,       0.0625,  0], [-1.5708,  0,       0      ]]]

def _R(x, y, z):
    Rx = np.array([[1,0,0],[0,np.cos(x),-np.sin(x)],[0,np.sin(x),np.cos(x)]])
    Ry = np.array([[np.cos(y),0,np.sin(y)],[0,1,0],[-np.sin(y),0,np.cos(y)]])
    Rz = np.array([[np.cos(z),-np.sin(z),0],[np.sin(z),np.cos(z),0],[0,0,1]])
    return Rz @ Ry @ Rx

def _T(xyz, rpy):
    T = np.eye(4); T[:3,:3] = _R(*rpy); T[:3,3] = xyz; return T

def quat_to_R(w, x, y, z):
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-w*z),  2*(x*z+w*y)],
        [  2*(x*y+w*z),1-2*(x*x+z*z),  2*(y*z-w*x)],
        [  2*(x*z-w*y),  2*(y*z+w*x),1-2*(x*x+y*y)]])

def R_to_quat(R):
    t = R[0,0]+R[1,1]+R[2,2]
    if t > 0:
        s = 0.5/np.sqrt(t+1); return np.array([0.25/s,(R[2,1]-R[1,2])*s,(R[0,2]-R[2,0])*s,(R[1,0]-R[0,1])*s])
    if R[0,0]>R[1,1] and R[0,0]>R[2,2]:
        s = 2*np.sqrt(1+R[0,0]-R[1,1]-R[2,2]); return np.array([(R[2,1]-R[1,2])/s,0.25*s,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s])
    if R[1,1]>R[2,2]:
        s = 2*np.sqrt(1+R[1,1]-R[0,0]-R[2,2]); return np.array([(R[0,2]-R[2,0])/s,(R[0,1]+R[1,0])/s,0.25*s,(R[1,2]+R[2,1])/s])
    s = 2*np.sqrt(1+R[2,2]-R[0,0]-R[1,1]); return np.array([(R[1,0]-R[0,1])/s,(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,0.25*s])

def FK(theta):
    zs, ps = [], []
    T = np.eye(4)
    for i, j in enumerate(joints):
        T = T @ _T(j[0], j[1])
        zs.append(T[:3,2].copy()); ps.append(T[:3,3].copy())
        T = T @ _T([0,0,0],[0,0,theta[i]])
    return T, T[:3,3].copy(), R_to_quat(T[:3,:3]), zs, ps

class IKTest(Node):
    def __init__(self):
        super().__init__('ik_test')
        self.sub = self.create_subscription(Pose, '/target_pose', self._cb, 10)
        # self.pub = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self.pub = self.create_publisher(JointTrajectory, '/sim_lite6_traj_controller/joint_trajectory', 10)
        self.theta = np.array([0.0, 0.7, 1.57, 0.0, 0.87, 0.0])

        lim_min = np.array([-6.28, -2.61, -0.06, -6.28, -2.16, -6.28])
        lim_max = np.array([ 6.28,  2.61,  5.23,  6.28,  2.16,  6.28])
        self.lim = (lim_min, lim_max)

    def _cb(self, msg: Pose):
        target_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        target_R   = quat_to_R(msg.orientation.w, msg.orientation.x,
                               msg.orientation.y, msg.orientation.z)

        theta = self.theta.copy()
        lim_min, lim_max = self.lim

        for step in range(500):
            T, pos, quat, zs, ps = FK(theta)
            R_err = target_R @ T[:3,:3].T
            dP = target_pos - pos
            dW = np.array([R_err[2,1]-R_err[1,2], R_err[0,2]-R_err[2,0], R_err[1,0]-R_err[0,1]]) * 0.5
            dX = np.append(dP, dW)

            if np.linalg.norm(dX) < 1e-3:
                self.get_logger().info(f"[OK] step={step} quat={np.round(quat,3)} pos={np.round(pos,3)}")
                break

            J = np.zeros((6,6))
            for i in range(6):
                r = pos - ps[i]
                J[:3,i] = np.cross(zs[i], r)
                J[3:,i] = zs[i]

            manip  = np.sqrt(abs(np.linalg.det(J @ J.T)))
            lam    = 0.1 + 0.4 * np.exp(-manip / 0.05)
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(6))
            theta  = np.clip(theta + 0.5 * np.clip(J_pinv @ dX, -0.1, 0.1), lim_min, lim_max)
        else:
            T, pos, quat, _, _ = FK(theta)
            self.get_logger().warn(f"[FAIL step=500] final quat={np.round(quat,3)} err={np.linalg.norm(dX):.4f}")

        delta    = float(np.max(np.abs(theta - self.theta)))

        move_sec  = max(3.0, float(delta) * 100.0)  # motor speed parameter

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
    node = IKTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()