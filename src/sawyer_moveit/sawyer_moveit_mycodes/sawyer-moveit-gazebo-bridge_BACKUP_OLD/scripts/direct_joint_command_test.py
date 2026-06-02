#!/usr/bin/env python3

import rospy
import intera_interface
from sensor_msgs.msg import JointState
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

rospy.init_node("direct_joint_command_test")

print("Getting robot state...")
rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)

print("Enabling robot...")
rs.enable()
rospy.sleep(1.0)

pub = rospy.Publisher(
    "/robot/limb/right/joint_command",
    JointCommand,
    queue_size=10
)

state = rospy.wait_for_message("/robot/joint_states", JointState)
current = dict(zip(state.name, state.position))

target = [current[name] for name in JOINTS]

# movimiento pequeño y seguro
target[0] += 0.20

cmd = JointCommand()
cmd.mode = JointCommand.POSITION_MODE
cmd.names = JOINTS
cmd.position = target

rate = rospy.Rate(50)

rospy.loginfo("Enviando comando directo a Sawyer Gazebo...")

start = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < 5.0:
    cmd.header.stamp = rospy.Time.now()
    pub.publish(cmd)
    rate.sleep()

rospy.loginfo("Test directo finalizado.")
