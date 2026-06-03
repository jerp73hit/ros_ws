#!/usr/bin/env python3

import sys
import rospy
import tf2_ros
import tf2_geometry_msgs
import moveit_commander

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped


def gazebo_pose_to_base(model_name):
    rospy.wait_for_service("/gazebo/get_model_state")
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    res = get_model_state(model_name, "world")

    if not res.success:
        raise RuntimeError("No se pudo obtener {}: {}".format(model_name, res.status_message))

    pose_world = PoseStamped()
    pose_world.header.frame_id = "world"
    pose_world.header.stamp = rospy.Time(0)
    pose_world.pose = res.pose

    pose_base = tf_buffer.transform(pose_world, "base", rospy.Duration(3.0))
    return pose_base


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("sync_scene_from_gazebo")

scene = moveit_commander.PlanningSceneInterface()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

rospy.sleep(2.0)

# Limpiar objetos previos
for obj in ["cafe_table_top", "block", "table", "cafe_table", "table_top", "cube"]:
    scene.remove_world_object(obj)

rospy.sleep(1.0)

# Obtener poses reales desde Gazebo y transformarlas a base
block_pose_base = gazebo_pose_to_base("block")
table_pose_base = gazebo_pose_to_base("cafe_table")

block_size = 0.05
table_thickness = 0.04

# El centro del cubo está sobre la mesa.
# La superficie de la mesa está justo debajo del cubo.
table_top_z = block_pose_base.pose.position.z - block_size / 2.0
table_center_z = table_top_z - table_thickness / 2.0

# Mesa en frame base
table_collision_pose = PoseStamped()
table_collision_pose.header.frame_id = "base"
table_collision_pose.pose.orientation.w = 1.0
table_collision_pose.pose.position.x = table_pose_base.pose.position.x
table_collision_pose.pose.position.y = table_pose_base.pose.position.y
table_collision_pose.pose.position.z = table_center_z

scene.add_box(
    "cafe_table_top",
    table_collision_pose,
    size=(0.90, 0.90, table_thickness)
)

# Bloque en frame base
block_collision_pose = PoseStamped()
block_collision_pose.header.frame_id = "base"
block_collision_pose.pose.orientation.w = 1.0
block_collision_pose.pose.position.x = block_pose_base.pose.position.x
block_collision_pose.pose.position.y = block_pose_base.pose.position.y
block_collision_pose.pose.position.z = block_pose_base.pose.position.z

scene.add_box(
    "block",
    block_collision_pose,
    size=(block_size, block_size, block_size)
)

rospy.sleep(2.0)

print("\n========== MOVEIT SCENE SYNC ==========")
print("Mesa base:")
print("x:", table_collision_pose.pose.position.x)
print("y:", table_collision_pose.pose.position.y)
print("z:", table_collision_pose.pose.position.z)

print("\nBloque base:")
print("x:", block_collision_pose.pose.position.x)
print("y:", block_collision_pose.pose.position.y)
print("z:", block_collision_pose.pose.position.z)

print("\nObjetos conocidos:", scene.get_known_object_names())
