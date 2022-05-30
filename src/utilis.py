import numpy as np
from math import sin, cos
from scipy.spatial.transform import Rotation as R


# 获取高度，
# 输入激光雷达读数，无人机的pitch和roll（弧度制）
# 输出无人机的高度

def get_height(value, roll, pitch):
    v = np.array([0, 0, value])

    cr, sr = cos(roll), sin(roll)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

    cp, sp = cos(pitch), sin(pitch)
    ry = np.array([[cp, 0, -sp], [0, 1, 0], [sp, 0, cp]])

    rm = np.dot(rx, ry)
    rv = np.dot(rm, v)

    return rv[2]


def quaternion_to_euler(q_fcu):
    r = R.from_quat(q_fcu)
    return r.as_euler('xyz', degrees=False)


def spin_job():
    rospy.spin()
