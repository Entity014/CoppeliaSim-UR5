import time
import numpy as np

pi = np.pi
d2r = pi / 180
r2d = 1 / d2r


# DH-table
def dh_theta(theta):
    dh_table = np.array(
        [
            [0, 0, 0.0892, -90],
            [90, 0, 0, 90],
            [0, 0.4251, 0, 0],
            [0, 0.39215, 0.11, -90],
            [-90, 0, 0.09473, 0],
            [90, 0, 0, 0],
        ]
    )
    for index, value in enumerate(dh_table):
        value[3] += theta[index]
    return dh_table


def homo_trans(dh_table, start, stop):
    T_init = np.eye(4)
    T_cal = np.eye(4)
    t_6e = np.eye(4)
    t_6e[2, 3] = 0.431
    for i in range(start, stop):
        alpha = np.radians(dh_table[i][0])
        theta = np.radians(dh_table[i][3])
        T_cal[0, :] = [
            np.cos(theta),
            -np.sin(theta),
            0,
            dh_table[i][1],
        ]
        T_cal[1, :] = [
            np.sin(theta) * np.cos(alpha),
            np.cos(theta) * np.cos(alpha),
            -np.sin(alpha),
            -np.sin(alpha) * dh_table[i][2],
        ]
        T_cal[2, :] = [
            np.sin(theta) * np.sin(alpha),
            np.cos(theta) * np.sin(alpha),
            np.cos(alpha),
            np.cos(alpha) * dh_table[i][2],
        ]
        T_cal[3, :] = [
            0,
            0,
            0,
            1,
        ]
        T_init = T_init @ T_cal
    return T_init @ t_6e


def homo_data(homo_mat):
    pos = homo_mat[:3, 3]
    rot = homo_mat[:3, :3]
    ori = np.array(
        [
            np.arctan2(-homo_mat[1, 2], homo_mat[2, 2]),
            np.arcsin(homo_mat[0, 2]),
            np.arctan2(-homo_mat[0, 1], homo_mat[0, 0]),
        ]
    )
    return pos, rot, ori


def find_jacobian(dh_table, type_joint=np.zeros(6)):
    """
    type_joint :
    0 is Revolut Joint
    1 is Prismatic Joint
    """
    j_array = []
    for index in range(len(dh_table)):
        homo_mat = homo_trans(dh_table, index, len(dh_table))
        k_0i = homo_data(homo_mat)[1] @ np.array([0, 0, 1])
        r_ie = homo_data(homo_mat)[0]
        j_1 = ((1 - type_joint[index]) * np.cross(k_0i, r_ie)) + (
            type_joint[index] * k_0i
        )
        j_2 = (1 - type_joint[index]) * k_0i
        j_array.append(j_1 + j_2)
    return np.array(j_array).T


def inverse_kinematic(
    hdl_j,
    end_pos,
    init_theta,
    tolerance=1e-4,
    alpha=0.1,
):
    th = {}
    for i in range(0, 6):
        th[i] = round(sim.getJointPosition(hdl_j[i]) * r2d, 2)
        init_theta[i] = np.deg2rad(th[i])

    dh_table = dh_theta(th)
    fk = homo_trans(dh_table, 0, 6)
    pos, rot, ori = homo_data(fk)
    init_pos = np.hstack((pos, ori))
    jacobian = find_jacobian(dh_table)
    jacobian_inv = np.linalg.pinv(jacobian)
    error_pos = init_pos - end_pos
    error_pos[1] = init_pos[1] - end_pos[1]
    # print(error_pos)
    deltha_theta = alpha * (jacobian_inv @ error_pos[:3])
    init_theta += deltha_theta
    if np.mean(error_pos) < tolerance:
        return init_theta
    return init_theta


def path_planning(hdl_j, positions, speeds, loop=False):
    theta = np.zeros(6)
    while True:
        for count in range(len(positions) - 1):
            cubic_polynomial(
                hdl_j,
                positions[count],
                positions[count + 1],
                speeds[count],
                theta,
            )


def cubic_polynomial(
    hdl_j, init_pos, end_pos, end_time, theta, end=False, init_speed=0, end_speed=0
):
    print(f"go to {end_pos}")
    a0 = init_pos
    a1 = init_speed
    a2 = (
        (3 / (end_time**2)) * (end_pos - init_pos)
        - ((3 / end_time) * init_speed)
        - ((1 / end_time) * end_speed)
    )
    a3 = ((-2 / (end_time**3)) * (end_pos - init_pos)) + (
        (1 / (end_time**2)) * (end_speed + 2 * init_speed)
    )
    start_t = time.time()
    while True:
        if time.time() - start_t > end_time:
            if end == False:
                break
            for count, hdl in enumerate(hdl_j):
                if count == 5:
                    sim.setJointTargetPosition(hdl, 0)
                else:
                    sim.setJointTargetPosition(hdl, theta_bc[count])
        else:
            t = time.time() - start_t
            ut = a0 + a1 * t + a2 * (t**2) + a3 * (t**3)
            theta = inverse_kinematic(hdl_j, ut, theta)
            for count, hdl in enumerate(hdl_j):
                if count == 5:
                    sim.setJointTargetPosition(hdl, 0)
                else:
                    sim.setJointTargetPosition(hdl, theta[count])
            theta_bc = theta

        sim.switchThread()


def sysCall_init():
    sim = require("sim")


def sysCall_thread():
    # define handles for axis
    hdl_j = []
    hdl_j.append(sim.getObject("/UR5/joint"))
    hdl_j.append(sim.getObject("/UR5/joint/link/joint"))
    hdl_j.append(sim.getObject("/UR5/joint/link/joint/link/joint"))
    hdl_j.append(sim.getObject("/UR5/joint/link/joint/link/joint/link/joint"))
    hdl_j.append(
        sim.getObject("/UR5/joint/link/joint/link/joint/link/joint/link/joint")
    )
    hdl_j.append(
        sim.getObject(
            "/UR5/joint/link/joint/link/joint/link/joint/link/joint/link/joint"
        )
    )
    hdl_end = sim.getObject("/UR5/EndPoint")

    new_theta = np.zeros(6)

    speeds = [10, 10, 10, 10]

    positions = [
        [-0.4, 0.4, 0.2, 0, 0, 0],
        [-0.5, 0.4, 0.2, 0, 0, 0],
        [-0.7, 0.4, 0.4, 0, 0, 0],
        [-0.5, 0.4, 0.6, 0, 0, 0],
        [-0.7, 0.4, 0.2, 0, 0, 0],
    ]

    path_planning(hdl_j, np.array(positions), speeds)

    # while True:

    #     box_arr = np.hstack((box_pos, box_ori))
    #     new_theta = inverse_kinematic(hdl_j, box_arr, new_theta)
    #     for i in range(0, 6):
    #         sim.setJointTargetPosition(hdl_j[i], new_theta[i])

    #     sim.switchThread()
    pass
