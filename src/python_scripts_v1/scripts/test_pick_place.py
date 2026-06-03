#!/usr/bin/env python3
import rospy
from llm_api import init_api, make_scene, adjust_grasp_waypoints, execute_waypoints

init_api()
rospy.sleep(1.0)

scene = make_scene()
t = scene["objects"]["papas_fritas"]["center"]

hover = [t[0], t[1], t[2] + 0.15]
grasp = [t[0], t[1], t[2]]

waypoints = [
    {"pos": hover, "ori": [180, 0, 0], "gripper": "open"},
    {"pos": hover, "ori": [180, 0, 180], "gripper": "open"},
    {"pos": grasp, "ori": [180, 0, 180], "gripper": "close"},
    {"pos": hover, "ori": [180, 0, 180], "gripper": "close"},
    {"pos": grasp, "ori": [180, 0, 180], "gripper": "open"},
    {"pos": hover, "ori": [180, 0, 180], "gripper": "open"},
]

adjusted = adjust_grasp_waypoints(scene, waypoints)
execute_waypoints(adjusted)
