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
    return T_init


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


def find_jacobian(dh_table, end_mat, type_joint):
    """
    type_joint :
    0 is Revolut Joint
    1 is Prismatic Joint
    """
    j_array = []
    for index in range(len(dh_table)):
        homo_mat = homo_trans(dh_table, index, len(dh_table)) @ end_mat
        k_0i = homo_data(homo_mat)[1] @ np.array([0, 0, 1])
        r_ie = homo_data(homo_mat)[0]
        j_1 = ((1 - type_joint[index]) * np.cross(k_0i, r_ie)) + (
            type_joint[index] * k_0i
        )
        j_2 = (1 - type_joint[index]) * k_0i
        j_array.append(j_1 + j_2)
    return np.array(j_array).T


def inverse_kinematic(init_pos, end_pos, init_theta, jacobian, alpha=0.05):
    error_pos = init_pos - end_pos
    jacobian_inv = np.linalg.pinv(jacobian)
    deltha_theta = jacobian_inv @ error_pos[:3]
    new_theta = init_theta + (alpha * deltha_theta)
    # new_theta = np.zeros(6)
    return new_theta


def sysCall_init():
    sim = require("sim")


def sysCall_thread():
    # define handles for axis
    hdl_j = {}
    hdl_j[0] = sim.getObject("/UR5/joint")
    hdl_j[1] = sim.getObject("/UR5/joint/link/joint")
    hdl_j[2] = sim.getObject("/UR5/joint/link/joint/link/joint")
    hdl_j[3] = sim.getObject("/UR5/joint/link/joint/link/joint/link/joint")
    hdl_j[4] = sim.getObject("/UR5/joint/link/joint/link/joint/link/joint/link/joint")
    hdl_j[5] = sim.getObject(
        "/UR5/joint/link/joint/link/joint/link/joint/link/joint/link/joint"
    )
    hdl_end = sim.getObject("/UR5/EndPoint")

    box = sim.getObject("/Cuboid")

    t = 0
    t1 = time.time()
    th = {}

    # Forward kinematics
    t_6e = np.eye(4)
    t_6e[2, 3] = 0.431
    t_6e = t_6e
    fk = np.eye(4)
    t_matrix = t_6e
    jacobian = np.array([])
    type_joints_arr = np.zeros(6)
    new_theta = np.zeros(6)

    while True:
        # p = 45 * pi / 180 * np.sin(0.2 * pi * t)
        for i in range(0, 6):
            th[i] = round(sim.getJointPosition(hdl_j[i]) * r2d, 2)
            new_theta[i] = np.deg2rad(th[i])

        # My Functions
        dh_table = dh_theta(th)
        fk = homo_trans(dh_table, 0, 6) @ t_matrix
        pos, rot, ori = homo_data(fk)
        jacobian = find_jacobian(dh_table, t_matrix, type_joints_arr)

        end_pos = sim.getObjectPosition(hdl_end, -1)
        box_pos = sim.getObjectPosition(box, -1)
        # get Euler's angle X-Y-Z
        end_ori = sim.getObjectOrientation(hdl_end, -1)
        box_ori = sim.getObjectOrientation(box, -1)

        init_pos = np.hstack((pos, ori))
        box_arr = np.hstack((box_pos, box_ori))
        new_theta = inverse_kinematic(init_pos, box_arr, new_theta, jacobian)
        for i in range(0, 6):
            sim.setJointTargetPosition(hdl_j[i], new_theta[i])

        print("-----------------------")
        print("Joint Position: {}".format(th))
        print("End point position < Program > : {}".format(np.array(end_pos).round(4)))
        print(f"End point position < Calculation > : {pos}")
        print(f"End point position < Error > : {pos - np.array(box_pos)}")
        print(f"End point orientation < Calculation > : {ori}")
        print("Box position < Program > : {}".format(np.array(box_pos).round(4)))
        print("Box orientation < Program > : {}".format(np.array(box_ori).round(4)))

        # time
        t = time.time() - t1

        sim.switchThread()  # resume in next simulation step
    pass


# See the user manual or the available code snippets for additional callback functions and details
