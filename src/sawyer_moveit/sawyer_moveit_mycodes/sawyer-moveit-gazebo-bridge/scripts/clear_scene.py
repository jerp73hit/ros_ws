#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("clear_scene")

scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(2.0)

for obj in ["cafe_table_top", "block", "table", "cafe_table"]:
    scene.remove_world_object(obj)

rospy.sleep(2.0)
print("Planning scene limpiada.")
