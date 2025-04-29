import redis
import numpy as np 
import time
import json

MATRIX_KEY = 'sai::interfaces::tutorial::matrix_key'


def xyz_fixed_angles_to_mat(alpha, beta, gamma):
    def rot_x(theta):
        return np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])

    def rot_y(theta):
        return np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

    def rot_z(theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])

    return rot_z(gamma) @ rot_y(beta) @ rot_x(alpha)

def mat_to_xyz_fixed_angles(R):
    c_beta = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    s_beta = -R[2, 0]
    if np.abs(c_beta ** 2) < 1e-10:
        """
        Singularity. Assuming alpha = 0, we get a matrix of the form:
            0   sin(beta) * sin(gamma)    sin(beta) * cos(gamma)
            0          cos(gamma)                -sin(gamma)
        -sin(beta)         0                         0
        We can find gamma by arctan2(-r_23, r_22).
        We can then find beta by taking the arcsin.
        """
        alpha = 0
        gamma = np.arctan2(-R[1, 2], R[1, 1])
        beta = np.arcsin(-R[2, 0])

    else:
        # normal case
        alpha = np.arctan2(R[1, 0] / c_beta, R[0, 0] / c_beta)
        beta = np.arctan2(s_beta, c_beta)
        gamma = np.arctan2(R[2, 1] / c_beta, R[2, 2] / c_beta)

    return (alpha, beta, gamma)


if __name__ == '__main__':
    print('Writing keys...')
    r = redis.Redis()

    theta = np.random.random((3,)) * (np.pi)
    print(theta)
    orig_rot_mat = xyz_fixed_angles_to_mat(*theta)
    r.set(MATRIX_KEY, str(orig_rot_mat.tolist()))

    while True:
        rot_mat = np.array(json.loads(r.get(MATRIX_KEY)))
        print(mat_to_xyz_fixed_angles(rot_mat))
        time.sleep(1)
