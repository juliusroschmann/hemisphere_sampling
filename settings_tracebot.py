#!/usr/bin/env python3
import os

"""
Moveit config params
"""
GROUP_NAME_ARM = "tracebot_right_arm"  # "tracebot_arms"
GROUP_NAME_GRIPPER = "tracebot_right_gripper"
WORLD_FRAME = "tracebot_base_link"  # "world"
EEF_LINK = "tracebot_right_arm_tcp"
PLANNING_TIME = 5
ROBOT_SPEED = 1 #0-1

"""
Topic definitions
"""
POSE_ARRAY_ROS_TOPIC = "/jr_hemisphere_view_points_pose_array"
MIDDLE_POINT_HEMISPHERE_TOPIC = "/jr_hemisphere_middle_point"
HEMISPHERE_POINT_AFTER_POINT_TOPIC = "/jr_hemisphere_point_after_point"

"""
Path to file for icoshpere generation
"""
NAME_OF_PYTHON_SCRIPT_TO_CALL = os.getcwd() + "/ico_tracebot.py"

"""
Coordinates of middle point of hemisphere
"""
POI_X = 0.25
POI_Y = -0.5
POI_Z = 1.4

# POI_X = -0.44
# POI_Y = 0.19
# POI_Z = 0.83

"""
Icosphere params
"""
SUB_DIVISIONS = 3
RADIUS = 0.44

"""
Collision Box dimensions
"""
FACTOR = 1.2
BOX_X = RADIUS / FACTOR
BOX_Y = RADIUS / FACTOR
BOX_Z = RADIUS / FACTOR
