import math

def quaternion_to_rpy(qx, qy, qz, qw):
    """Equivalent to tf2::Matrix3x3(tf2::Quaternion).getRPY(roll,pitch,yaw)."""
    r11 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r21 = 2.0 * (qx * qy + qz * qw)
    r31 = 2.0 * (qx * qz - qy * qw)
    r32 = 2.0 * (qy * qz + qx * qw)
    r33 = 1.0 - 2.0 * (qx * qx + qy * qy)

    roll = math.atan2(r32, r33)
    pitch = math.asin(-r31)
    yaw = math.atan2(r21, r11)
    return roll, pitch, yaw


def rpy_to_quaternion(roll, pitch, yaw):
    """Equivalent to tf2::Quaternion.setRPY(roll,pitch,yaw)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw