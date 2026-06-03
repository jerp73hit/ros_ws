#!/usr/bin/env python3
import rospy
from llm_api import init_api, make_scene, adjust_grasp_waypoints, execute_waypoints

init_api()
rospy.sleep(1.0)
scene = make_scene()

# 2. Extract locations and dimensions for the coke_can and the bowl
coke_can = scene["objects"]["coke_can"]
bowl = scene["objects"]["bowl"]

pick_x, pick_y, pick_z = coke_can["center"]

# Calculate the placement position on top of the bowl
# Using the bowl's center XY and adding its Z half-extent to find its top surface
place_x = bowl["center"][0]
place_y = bowl["center"][1]
place_z = bowl["center"][2] + bowl["half_extents"][2]

# 3. Define the physical movement points
# Hovering 15cm above objects prevents lateral collisions during travel
pick_hover = [pick_x, pick_y, pick_z + 0.15]
pick_grasp = [pick_x, pick_y, pick_z]

# Hover above the bowl before descending to place it
place_hover = [place_x, place_y, place_z + 0.15]
place_release = [place_x, place_y, place_z + 0.02] # Add slight 2cm clearance for safe release

# 4. Construct the waypoint sequence
# Approach -> Grasp -> Ascend -> Move over destination -> Descend -> Release -> Ascend
waypoints = [
    {"pos": pick_hover, "ori": [180, 0, 180], "gripper": "open"},
    {"pos": pick_grasp, "ori": [180, 0, 180], "gripper": "close"},
    {"pos": pick_hover, "ori": [180, 0, 180], "gripper": "close"},
    {"pos": place_hover, "ori": [180, 0, 180], "gripper": "close"},
    {"pos": place_release, "ori": [180, 0, 180], "gripper": "open"},
    {"pos": place_hover, "ori": [180, 0, 180], "gripper": "open"}
]

# 5. Snap grasp waypoints to exact geometry and execute
adjusted = adjust_grasp_waypoints(scene, waypoints)
execute_waypoints(adjusted)

# init_api()
# rospy.sleep(1.0)
#
# scene = make_scene()
# t = scene["objects"]["strawberry"]["center"]
#
# hover = [t[0], t[1], t[2] + 0.15]
# grasp = [t[0], t[1], t[2]]
# hover2 = [t[0], t[1] - 0.15, t[2] + 0.15]
# grasp2 = [t[0], t[1] - 0.15, t[2]]
#
# waypoints = [
#     {"pos": hover, "ori": [180, 0, 0], "gripper": "open"},
#     {"pos": hover, "ori": [180, 0, 180], "gripper": "open"},
#     {"pos": grasp, "ori": [180, 0, 180], "gripper": "close"},
#     {"pos": hover, "ori": [180, 0, 180], "gripper": "close"},
#     {"pos": hover2, "ori": [180, 0, 180], "gripper": "close"},
#     {"pos": grasp2, "ori": [180, 0, 180], "gripper": "open"},
#     {"pos": hover2, "ori": [180, 0, 180], "gripper": "open"},
# ]
#
# adjusted = adjust_grasp_waypoints(scene, waypoints)
# execute_waypoints(adjusted)
