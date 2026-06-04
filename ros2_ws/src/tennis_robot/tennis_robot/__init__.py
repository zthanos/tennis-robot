import math


def yaw_from_quaternion(q) -> float:
    """Extract yaw (rotation around Z) from a ROS geometry_msgs/Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)
