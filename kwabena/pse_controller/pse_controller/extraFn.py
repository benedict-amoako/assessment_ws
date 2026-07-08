import math


def quaternion_to_yaw(q):
    """
    Convert a ROS quaternion into a yaw angle (radians).
    """

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    """
    Normalize an angle to the range [-pi, pi].
    """

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle