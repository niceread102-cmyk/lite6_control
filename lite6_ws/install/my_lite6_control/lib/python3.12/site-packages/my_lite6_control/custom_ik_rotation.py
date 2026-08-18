import numpy as np

np.set_printoptions(precision=3, suppress=True)

L_MIN = np.array([-6.28, -2.61, -0.06, -6.28, -2.16, -6.28])
L_MAX = np.array([ 6.28,  2.61,  5.23,  6.28,  2.16,  6.28])

TCP_OFFSET = 0.0836  # link_eef -> gripper TCP, along local Z (uflite_gripper.urdf.xacro joint_tcp)

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
                   [0,          0,         1]])
    return Rz @ Ry @ Rx

def make_transform(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = euler_rotation_matrix(*rpy)
    T[:3,  3] = xyz
    return T

def rotation_to_quaternion(R):
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return np.array([0.25/s, 
                       (R[2,1]-R[1,2])*s, 
                       (R[0,2]-R[2,0])*s, 
                       (R[1,0]-R[0,1])*s])

    if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return np.array([(R[2,1]-R[1,2])/s,
                         0.25*s, 
                         (R[0,1]+R[1,0])/s, 
                         (R[0,2]+R[2,0])/s])

    if R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return np.array([(R[0,2]-R[2,0])/s,
                         (R[0,1]+R[1,0])/s,
                         0.25*s,
                         (R[1,2]+R[2,1])/s])

    s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])

    return np.array([(R[1,0]-R[0,1])/s, 
                     (R[0,2]+R[2,0])/s, 
                     (R[1,2]+R[2,1])/s, 
                     0.25*s])

def quat_to_R(w, x, y, z):
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])

def FK(theta):
    z_axes_local = []
    positions_local = []
    T_total = np.eye(4)

    for i, joint in enumerate(joints):
        T_total = T_total @ make_transform(joint[0], joint[1])
        z_axes_local.append(T_total[:3, 2].copy())
        positions_local.append(T_total[:3, 3].copy())
        T_total = T_total @ make_transform([0,0,0], [0, 0, theta[i]])

    T_total = T_total @ make_transform([0, 0, TCP_OFFSET], [0, 0, 0])
    pos  = T_total[:3, 3]
    quat = rotation_to_quaternion(T_total[:3, :3])
    return T_total, pos, quat, z_axes_local, positions_local

def solve_ik(target_pos, target_R, theta_init):
    theta     = theta_init.copy()
    theta_ref = theta_init.copy()
    for step in range(2000):
        T, pos, quat, zs, ps = FK(theta)
        R_error = target_R @ T[:3, :3].T
        dP = target_pos - pos
        trace = R_error[0,0] + R_error[1,1] + R_error[2,2]
        if trace <= -0.999:
            if R_error[0,0] >= R_error[1,1] and R_error[0,0] >= R_error[2,2]:
                ax = np.array([R_error[0,0]+1, R_error[1,0], R_error[2,0]])
            elif R_error[1,1] >= R_error[2,2]:
                ax = np.array([R_error[0,1], R_error[1,1]+1, R_error[2,1]])
            else:
                ax = np.array([R_error[0,2], R_error[1,2], R_error[2,2]+1])
            dW = np.pi * ax / (np.linalg.norm(ax) + 1e-9)
        else:
            dW = np.array([R_error[2,1]-R_error[1,2],
                           R_error[0,2]-R_error[2,0],
                           R_error[1,0]-R_error[0,1]]) * 0.5

        dX = np.append(dP, dW)
        if np.linalg.norm(dX) < 5e-3:
            for i in [0, 3, 5]:
                diff      = theta[i] - theta_ref[i]
                theta[i]  = np.clip(theta_ref[i] + np.arctan2(np.sin(diff), np.cos(diff)), L_MIN[i], L_MAX[i])
            print(f"[Success] step={step} pos={np.round(pos,3)} quat={np.round(quat,3)}")
            return theta, True
        J = np.zeros((6, 6))

        for i in range(6):
            r = pos - ps[i]
            J[:3, i] = np.cross(zs[i], r)
            J[3:, i] = zs[i]
        manip  = np.sqrt(abs(np.linalg.det(J @ J.T)))
        lam    = 0.1 + 0.4 * np.exp(-manip / 0.05)
        J_pinv = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(6))
        dtheta = np.clip(J_pinv @ dX, -0.1, 0.1)
        q_null = np.zeros(6)
        # q_null[3] = -0.5 * theta[3]
        # dtheta += (np.eye(6) - J_pinv @ J) @ q_null
        theta  = np.clip(theta + 0.5 * dtheta, L_MIN, L_MAX)
        for i in [0, 3, 5]:
            diff      = theta[i] - theta_ref[i]
            theta[i]  = np.clip(theta_ref[i] + np.arctan2(np.sin(diff), np.cos(diff)), L_MIN[i], L_MAX[i])
    print(f"[FAIL] step=2000 err={np.linalg.norm(dX):.4f}")
    return None, False

if __name__ == '__main__':
    theta_init = np.array([0.0, 0.7, 1.57, 0.0, 0.87, 0.0])
    target_pos = np.array([0.3, 0.0, 0.3])
    target_R   = euler_rotation_matrix(np.pi, 0, 0)
    theta, ok  = solve_ik(target_pos, target_R, theta_init)
    print(theta)