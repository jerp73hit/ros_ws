#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("add_table_scene")

scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(2.0)

frame = "base"

for obj in ["cafe_table_top", "block", "table", "cafe_table", "table_top",
            "cube", "mustard", "strawberry", "banana",
            "coke_can", "bowl", "planta_maceta","esponja_lavaplatos","papas_fritas"]:
    scene.remove_world_object(obj)
rospy.sleep(1.0)

# ── Z conversion reminder ─────────────────────────────────────────────────────
# Gazebo world frame:  Z=0 at ground, table surface at Z≈0.80
# MoveIt base frame:   Z=0 at robot base (0.93m above ground)
# base_z = world_z - 0.93
# table surface in base frame = 0.80 - 0.93 = -0.13
# object center in base frame = -0.13 + (object_height / 2)
# ─────────────────────────────────────────────────────────────────────────────

# ── Table ─────────────────────────────────────────────────────────────────────
table_pose = PoseStamped()
table_pose.header.frame_id = frame
table_pose.pose.position.x = 0.75
table_pose.pose.position.y = 0.0
table_pose.pose.position.z = -0.16
table_pose.pose.orientation.w = 1.0
scene.add_box("cafe_table_top", table_pose, size=(0.90, 0.90, 0.04))

# ── Block  (primary grasp target) ─────────────────────────────────────────────
# 5cm cube → center = -0.13 + 0.025 = -0.105
block_pose = PoseStamped()
block_pose.header.frame_id = frame
block_pose.pose.position.x = 0.55
block_pose.pose.position.y = -0.25
block_pose.pose.position.z = -0.105
block_pose.pose.orientation.w = 1.0
scene.add_box("block", block_pose, size=(0.05, 0.05, 0.05))

# ── Row 1 (y=+0.20) ───────────────────────────────────────────────────────────

# Mustard  (19cm tall, 5cm wide) → center = -0.13 + 0.095 = -0.035
mustard_pose = PoseStamped()
mustard_pose.header.frame_id = frame
mustard_pose.pose.position.x = 0.45
mustard_pose.pose.position.y = 0.20
mustard_pose.pose.position.z = -0.035
mustard_pose.pose.orientation.w = 1.0
scene.add_box("mustard", mustard_pose, size=(0.05, 0.05, 0.19))

# Coke can  (12cm tall, 6.6cm diameter) → center = -0.13 + 0.06 = -0.07
coke_can_pose = PoseStamped()
coke_can_pose.header.frame_id = frame
coke_can_pose.pose.position.x = 0.65
coke_can_pose.pose.position.y = 0.30
coke_can_pose.pose.position.z = -0.07
coke_can_pose.pose.orientation.w = 1.0
scene.add_box("coke_can", coke_can_pose, size=(0.07, 0.07, 0.12))

# Bowl  (8cm tall, 15cm diameter) → center = -0.13 + 0.04 = -0.09
bowl_pose = PoseStamped()
bowl_pose.header.frame_id = frame
bowl_pose.pose.position.x = 0.85
bowl_pose.pose.position.y = 0.20
bowl_pose.pose.position.z = -0.09
bowl_pose.pose.orientation.w = 1.0
scene.add_box("bowl", bowl_pose, size=(0.15, 0.15, 0.08))

# ── Row 2 (y=0.00) ────────────────────────────────────────────────────────────

# Banana  (4cm tall, 18cm long)
banana_pose = PoseStamped()
banana_pose.header.frame_id = frame
banana_pose.pose.position.x = 0.45
banana_pose.pose.position.y = 0.00
banana_pose.pose.position.z = -0.13 # Lowered Z value here
banana_pose.pose.orientation.w = 1.0
scene.add_box("banana", banana_pose, size=(0.18, 0.06, 0.04))

# Strawberry  (3cm tall, 4cm wide)
strawberry_pose = PoseStamped()
strawberry_pose.header.frame_id = frame
strawberry_pose.pose.position.x = 0.65
strawberry_pose.pose.position.y = 0.00
strawberry_pose.pose.position.z = -0.145 # Lowered Z value here
strawberry_pose.pose.orientation.w = 1.0
scene.add_box("strawberry", strawberry_pose, size=(0.04, 0.04, 0.03))

# Planta maceta  (assumed 10cm tall, 8cm wide) → center = -0.13 + 0.05 = -0.08
planta_maceta_pose = PoseStamped()
planta_maceta_pose.header.frame_id = frame
planta_maceta_pose.pose.position.x = 0.85
planta_maceta_pose.pose.position.y = 0.00
planta_maceta_pose.pose.position.z = -0.08
planta_maceta_pose.pose.orientation.w = 1.0
scene.add_box("planta_maceta", planta_maceta_pose, size=(0.08, 0.08, 0.10))

# esponja_lavaplatos (assumed 10cm tall, 8cm wide) → center = -0.13 + 0.05 = -0.08
esponja_lavaplatos_pose = PoseStamped()
esponja_lavaplatos_pose.header.frame_id = frame
esponja_lavaplatos_pose.pose.position.x = 0.45
esponja_lavaplatos_pose.pose.position.y = -0.30
esponja_lavaplatos_pose.pose.position.z = -0.08
esponja_lavaplatos_pose.pose.orientation.w = 1.0
scene.add_box("esponja_lavaplatos", esponja_lavaplatos_pose, size=(0.08, 0.08, 0.10))

# esponja_lavaplatos (assumed 10cm tall, 8cm wide) → center = -0.13 + 0.05 = -0.08
papas_fritas_pose = PoseStamped()
papas_fritas_pose.header.frame_id = frame
papas_fritas_pose.pose.position.x = 0.65
papas_fritas_pose.pose.position.y = -0.30
papas_fritas_pose.pose.position.z = -0.08
papas_fritas_pose.pose.orientation.w = 1.0
scene.add_box("papas_fritas", papas_fritas_pose, size=(0.08, 0.08, 0.10))

rospy.sleep(2.0)
print("Planning Scene updated in frame:", frame)
print("Objects:", scene.get_known_object_names())
