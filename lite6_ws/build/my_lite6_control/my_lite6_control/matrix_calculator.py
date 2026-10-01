import numpy as np

# gripper default rotation
R_EE = np.array([[1, 0, 0],
                 [0, -1, 0],
                 [0, 0, -1]])

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


def main():
    user_input = input("input target pos x y z roll pitch yaw (or type 'q' to quit): ")
                
    x, y, z, target_roll, target_pitch, target_yaw = map(float, user_input.split())

    num = euler_rotation_matrix(np.radians(target_roll), np.radians(target_pitch), np.radians(target_yaw))

    print(f"rotation {num}")

    delta_rotation = num @ R_EE.T

    print(f"d_R {delta_rotation}")

if __name__ == '__main__':
    main()