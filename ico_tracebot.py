#!/usr/bin/env python3

from math import atan2

import geometry_msgs.msg
import numpy as np
import pandas as pd
import rospy
from icosphere import icosphere
from scipy.spatial.transform import Rotation as R
from settings_tracebot import *


def create_ico(subdivisions, radius):
    """
    Creates a hemisphere given a number of subdivisions and radius.
    """
    vertices2, _ = icosphere(subdivisions)
    vertices2 *= radius
    return np.around(vertices2, decimals=4)


def sort_vertices_by_angle(vertices):
    """
    Sort the given vertices by angle.
    """
    vertices_sorted = sorted(vertices, key=lambda c: (
        atan2(c[0], c[1])), reverse=True)
    return np.array(vertices_sorted)


def select_positive_vertices(vertices_sorted):
    """
    Select the positive vertices in the given sorted vertices only.
    """
    return np.array([sublist for sublist in vertices_sorted if sublist[2] >= 0])


def calculate_z_vectors_from_middle_point_hemisphere(positive_vertices_sorted, MIDDLE_POINT_HEMISPHERE):
    """
    Calculate the z vectors for each vertex.
    """
    z_vectors_to_poi_3d = np.array([])
    for point_on_ico in positive_vertices_sorted:
        vector = MIDDLE_POINT_HEMISPHERE - point_on_ico
        vector = vector / np.linalg.norm(vector)
        z_vectors_to_poi_3d = np.append(z_vectors_to_poi_3d, vector)
    z_vectors_to_poi_3d = z_vectors_to_poi_3d.reshape((-1, 3))
    return z_vectors_to_poi_3d


def calculate_x_vectors_from_middle_point_hemisphere(z_vectors_to_poi_3d):
    """
    Calculate the x vectors for each vertex.
    """
    x_vectors_to_poi_3d = np.cross(z_vectors_to_poi_3d, [0.0, 0.0, 1.0])
    x_vectors_to_poi_3d[0][0] = 1.0
    return np.around(x_vectors_to_poi_3d, decimals=4)


def calculate_y_vectors_from_middle_point_hemisphere(z_vectors_to_poi_3d, x_vectors_to_poi_3d):
    """
    Calculate the y vectors for each vertex.
    """
    y_vectors_to_poi_3d = np.cross(z_vectors_to_poi_3d, x_vectors_to_poi_3d)
    return np.around(y_vectors_to_poi_3d, decimals=4)


def calculate_poses_6d(positive_vertices_sorted, x_vectors_to_poi_3d, y_vectors_to_poi_3d, z_vectors_to_poi_3d):
    """
    Calculate 6D poses for each vertex.
    """
    quats = np.array([])
    for vec in range(len(x_vectors_to_poi_3d)):
        rot_mat = np.matrix(
            [x_vectors_to_poi_3d[vec], y_vectors_to_poi_3d[vec], z_vectors_to_poi_3d[vec]]).T
        r = R.from_matrix(rot_mat)
        r = r.as_quat()
        quats = np.append(quats, r)

    quats = quats.reshape(-1, 4)
    return np.hstack((positive_vertices_sorted, quats))


def create_pose_array_msg(poses_6d):
    """
    Create a pose array message from the given 6D poses.
    """
    pose_array = geometry_msgs.msg.PoseArray()

    for pose in poses_6d:
        ros_pose = geometry_msgs.msg.Pose()
        ros_pose.position.x = pose[0]
        ros_pose.position.y = pose[1]
        ros_pose.position.z = pose[2]
        ros_pose.orientation.x = pose[3]
        ros_pose.orientation.y = pose[4]
        ros_pose.orientation.z = pose[5]
        ros_pose.orientation.w = pose[6]
        pose_array.poses.append(ros_pose)

    pose_array.header.frame_id = WORLD_FRAME
    return pose_array


def publish_middle_point_and_hemisphere(MIDDLE_POINT_HEMISPHERE, pose_array):
    """
    Publishes the given middle point and the 6d pose array representing the hemisphere.
    """
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = WORLD_FRAME

    pose.pose.position.x = MIDDLE_POINT_HEMISPHERE[0]
    pose.pose.position.y = MIDDLE_POINT_HEMISPHERE[1]
    pose.pose.position.z = MIDDLE_POINT_HEMISPHERE[2]

    pose.pose.orientation.w = 1.0

    for _ in range(10):
        middle_point_hemisphere_publisher.publish(pose)
        rospy.sleep(0.2)
        display_pose_array_publisher.publish(pose_array)
        rospy.sleep(0.2)
    return


if __name__ == '__main__':
    rospy.init_node('jr_display_pose_array')
    display_pose_array_publisher = rospy.Publisher(
        POSE_ARRAY_ROS_TOPIC,
        geometry_msgs.msg.PoseArray,
        queue_size=0,
    )

    middle_point_hemisphere_publisher = rospy.Publisher(
        MIDDLE_POINT_HEMISPHERE_TOPIC,
        geometry_msgs.msg.PoseStamped,
        queue_size=0,
    )

    MIDDLE_POINT_HEMISPHERE = np.array([POI_X, POI_Y, POI_Z])
    vertices_up_to_subdivs = []

    if SUB_DIVISIONS <= 1:
        vertices = create_ico(subdivisions=1, radius=RADIUS)
        vertices_up_to_subdivs.append(vertices)

    else:
        for it in range(1, SUB_DIVISIONS+1):
            tmp_vertices = create_ico(subdivisions=it, radius=RADIUS)
            vertices_up_to_subdivs.append(tmp_vertices)

        vertices = vertices_up_to_subdivs[-1]

    vertices_sorted = sort_vertices_by_angle(vertices)
    positive_vertices_sorted = select_positive_vertices(vertices_sorted)
    positive_vertices_sorted += MIDDLE_POINT_HEMISPHERE

    for idx in range(len(vertices_up_to_subdivs)):
        vertices_up_to_subdivs[idx] = sort_vertices_by_angle(
            vertices_up_to_subdivs[idx])
        vertices_up_to_subdivs[idx] = select_positive_vertices(
            vertices_up_to_subdivs[idx])
        vertices_up_to_subdivs[idx] += MIDDLE_POINT_HEMISPHERE

    tmp_pos_vert = positive_vertices_sorted
    # indexed_positive_vertices_sorted = np.array([])
    counter = 0
    vertices_indexed_dict = {"Idx": [], "SubDiv": [],
                             "Point": [], "Succ. Approached": [], "Error": []}

    for index in range(len(vertices_up_to_subdivs)):
        for vertex in tmp_pos_vert:
            itemindex = np.where(
                (vertex == vertices_up_to_subdivs[index]).all(axis=1))
            itemindex = itemindex[0]
            vertex = np.around(vertex, decimals=4)
            if itemindex.size == 1:
                vertices_indexed_dict["Idx"].append(counter)
                vertices_indexed_dict["SubDiv"].append(index)
                vertices_indexed_dict["Point"].append(vertex.tolist())
                vertices_indexed_dict["Succ. Approached"].append(0)
                vertices_indexed_dict["Error"].append("No execution tried yet")
                # indexed_positive_vertices_sorted = np.append(indexed_positive_vertices_sorted, np.insert(vertex, 0, index))
                # indexed_positive_vertices_sorted = np.append(indexed_positive_vertices_sorted, (np.array(index), vertex))
                tmp_pos_vert = tmp_pos_vert[1:]
                counter += 1

    # indexed_positive_vertices_sorted = indexed_positive_vertices_sorted.reshape((-1, 4))
    df_pose_array = pd.DataFrame.from_dict(vertices_indexed_dict)
    df_pose_array.to_csv("pose_array.csv", index=False)

    z_vectors_to_poi_3d = calculate_z_vectors_from_middle_point_hemisphere(
        positive_vertices_sorted, MIDDLE_POINT_HEMISPHERE)
    x_vectors_to_poi_3d = calculate_x_vectors_from_middle_point_hemisphere(
        z_vectors_to_poi_3d)
    y_vectors_to_poi_3d = calculate_y_vectors_from_middle_point_hemisphere(
        z_vectors_to_poi_3d, x_vectors_to_poi_3d)

    poses_6d = calculate_poses_6d(
        positive_vertices_sorted, x_vectors_to_poi_3d, y_vectors_to_poi_3d, z_vectors_to_poi_3d)
    print(type(poses_6d))
    print(poses_6d)
    # poses_6d_dict = {"x": [], "y": [],
    #                          "z": [], "qx": [], "qy": [], "qz": [], "qw": []}
    poses_6d_dict = ["x", "y", "z", "qx", "qy", "qz", "qw"]
    pd.DataFrame(poses_6d).to_csv("poses_6d.csv", header=poses_6d_dict, index=None)
    pose_array = create_pose_array_msg(poses_6d)

    input("============ Press `Enter` to publish the middle point and the viewing hemisphere!")
    publish_middle_point_and_hemisphere(MIDDLE_POINT_HEMISPHERE, pose_array)
