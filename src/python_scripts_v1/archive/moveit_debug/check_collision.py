#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("check_collision_state")

robot = moveit_commander.RobotCommander()

rospy.wait_for_service("/check_state_validity")
check_state = rospy.ServiceProxy("/check_state_validity", GetStateValidity)

current_state = robot.get_current_state()

req = GetStateValidityRequest()
req.robot_state = current_state
req.group_name = "right_arm"

res = check_state(req)

print("\n========== MOVEIT STATE VALIDITY ==========")
print("State valid:", res.valid)

if not res.valid:
    print("\nContacts detected:")
    for contact in res.contacts:
        print("- {}  <-->  {}".format(contact.contact_body_1, contact.contact_body_2))
else:
    print("No collision detected.")
