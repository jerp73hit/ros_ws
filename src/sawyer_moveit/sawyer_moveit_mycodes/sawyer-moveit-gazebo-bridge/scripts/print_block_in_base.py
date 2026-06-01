#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped


rospy.init_node("print_block_in_base")

rospy.wait_for_service("/gazebo/get_model_state")
get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

res = get_model_state("block", "world")

if not res.success:
    print("No se pudo obtener el modelo block desde Gazebo:")
    print(res.status_message)
    exit(1)

pose_world = PoseStamped()
pose_world.header.frame_id = "world"
pose_world.header.stamp = rospy.Time(0)
pose_world.pose = res.pose

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

rospy.sleep(1.0)

pose_base = tf_buffer.transform(pose_world, "base", rospy.Duration(3.0))

print("\n========== BLOCK POSE ==========")
print("Gazebo world:")
print("x:", pose_world.pose.position.x)
print("y:", pose_world.pose.position.y)
print("z:", pose_world.pose.position.z)

print("\nMoveIt/base:")
print("x:", pose_base.pose.position.x)
print("y:", pose_base.pose.position.y)
print("z:", pose_base.pose.position.z)
