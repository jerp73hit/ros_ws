#!/usr/bin/env python3

import sys
import rospy
import tf2_ros
import tf2_geometry_msgs
import moveit_commander

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped


def get_block_pose_base():
    rospy.wait_for_service("/gazebo/get_model_state")
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    res = get_model_state("block", "world")

    if not res.success:
        raise RuntimeError("No se pudo obtener block: {}".format(res.status_message))

    pose_world = PoseStamped()
    pose_world.header.frame_id = "world"
    pose_world.header.stamp = rospy.Time(0)
    pose_world.pose = res.pose

    pose_base = tf_buffer.transform(pose_world, "base", rospy.Duration(3.0))
    return pose_base


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_tip_above_block_from_gazebo")

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

rospy.sleep(1.0)

block_pose_base = get_block_pose_base()

group = moveit_commander.MoveGroupCommander("right_arm")
group.set_end_effector_link("right_gripper_tip")
group.set_pose_reference_frame("base")
group.set_start_state_to_current_state()

group.set_planning_time(20.0)
group.set_num_planning_attempts(50)
group.set_max_velocity_scaling_factor(0.08)
group.set_max_acceleration_scaling_factor(0.08)

group.set_goal_position_tolerance(0.03)

hover_distance = 0.18

target_position = [
    block_pose_base.pose.position.x,
    block_pose_base.pose.position.y,
    block_pose_base.pose.position.z + hover_distance
]

print("\n========== MOVE ABOVE BLOCK ==========")
print("Block in base:")
print("x:", block_pose_base.pose.position.x)
print("y:", block_pose_base.pose.position.y)
print("z:", block_pose_base.pose.position.z)

print("\nTarget gripper tip:")
print(target_position)

group.set_position_target(target_position, "right_gripper_tip")

success = group.go(wait=True)

group.stop()
group.clear_pose_targets()

if success:
    print("Gripper tip movido encima del bloque.")
else:
    print("No se pudo mover encima del bloque.")
