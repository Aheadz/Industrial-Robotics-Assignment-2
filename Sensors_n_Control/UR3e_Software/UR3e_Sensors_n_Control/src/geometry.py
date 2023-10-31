from geometry_msgs.msg import *
from tf.transformations import *


def matrix_from_point_msg(point):
    return translation_matrix((point.x, point.y, point.z))


def matrix_from_quaternion_msg(quaternion):
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    return quaternion_matrix(q)


def matrix_from_pose_msg(pose):
    t = matrix_from_point_msg(pose.position)
    r = matrix_from_quaternion_msg(pose.orientation)
    return concatenate_matrices(t, r)


def point_msg_from_matrix(transformation):
    msg = Point()
    msg.x = transformation[0][3]
    msg.y = transformation[1][3]
    msg.z = transformation[2][3]
    return msg


def quaternion_msg_from_matrix(transformation):
    q = quaternion_from_matrix(transformation)
    msg = Quaternion()
    msg.x = q[0]
    msg.y = q[1]
    msg.z = q[2]
    msg.w = q[3]
    return msg


def pose_msg_from_matrix(transformation):
    msg = Pose()
    msg.position = point_msg_from_matrix(transformation)
    msg.orientation = quaternion_msg_from_matrix(transformation)
    return msg


def translate_pose_msg(pose, x, y, z):
    initial = matrix_from_pose_msg(pose)
    transform = translation_matrix((x,y,z))
    return pose_msg_from_matrix(concatenate_matrices(initial, transform))


def rotate_pose_msg_by_euler_angles(pose, r, p, y):
    initial = matrix_from_pose_msg(pose)
    transform = quaternion_matrix(quaternion_from_euler(r, p, y))
    return pose_msg_from_matrix(concatenate_matrices(initial, transform))


def rotate_pose_msg_about_origin(pose, r, p, y):
    initial = matrix_from_pose_msg(pose)
    transform = quaternion_matrix(quaternion_from_euler(r, p, y))
    return pose_msg_from_matrix(concatenate_matrices(transform, initial))