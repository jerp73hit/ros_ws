#!/usr/bin/env python3

import rospy
import intera_interface
from intera_core_msgs.msg import JointCommand

JOINTS = [
    "right_j0",
    "right_j1",
    "right_j2",
    "right_j3",
    "right_j4",
    "right_j5",
    "right_j6",
]

SAFE_POSE = [
    -0.041662954890248294,
    -1.0258291091425074,
    0.0293680414401436,
    2.17518162913313,
    -0.06703022873354225,
    0.3968371433926965,
    1.7659649178699421,
]

rospy.init_node("send_safe_pose")

print("Enabling robot...")
rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
rs.enable()
rospy.sleep(1.0)

pub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size=10)

cmd = JointCommand()
cmd.mode = JointCommand.POSITION_MODE
cmd.names = JOINTS
cmd.position = SAFE_POSE

rate = rospy.Rate(50)

print("Sending safe pose...")
start = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < 6.0:
    cmd.header.stamp = rospy.Time.now()
    pub.publish(cmd)
    rate.sleep()

print("Done.")
