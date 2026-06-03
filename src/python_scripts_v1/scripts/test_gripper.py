#!/usr/bin/env python3

import rospy
import intera_interface

rospy.init_node("test_gripper")

print("Enabling robot...")
rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
rs.enable()
rospy.sleep(1.0)

print("Creating gripper interface...")
gripper = intera_interface.Gripper()

print("Opening gripper...")
gripper.open()
rospy.sleep(2.0)

print("Closing gripper...")
gripper.close()
rospy.sleep(2.0)

print("Opening gripper again...")
gripper.open()
rospy.sleep(2.0)

print("Gripper test finished.")
