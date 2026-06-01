#!/usr/bin/env python3
import rospy
import os
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

def delete_if_exists(model_name):
    try:
        rospy.wait_for_service("/gazebo/delete_model", timeout=2.0)
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model(model_name)
    except Exception:
        pass

def load_gazebo_models(
    table_pose=Pose(
        position=Point(x=0.75, y=0.0, z=0.0),
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    ),

    # ── Primary target ───────────────────────────────────────
    # Moved back to maintain >20cm distance from Row 2
    block_pose=Pose(
        position=Point(x=0.55, y=-0.25, z=0.82),
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    ),

    # ── Row 1  (y=+0.20) ────────────────────────────────────
    # 20cm spacing between centers in X and Y
    mustard_pose=Pose(
        position=Point(x=0.45, y=0.20, z=0.90),
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    ),
    coke_can_pose=Pose(
        position=Point(x=0.65, y=0.30, z=0.86),
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    ),
    bowl_pose=Pose(
        position=Point(x=0.85, y=0.20, z=0.84),
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    ),

    # ── Row 2  (y=0.00) ─────────────────────────────────────
    banana_pose=Pose(
        position=Point(x=0.45, y=0.00, z=0.80), # Z lowered from 0.82 to 0.80
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    ),
    strawberry_pose=Pose(
        position=Point(x=0.65, y=0.00, z=0.79), # Z lowered from 0.82 to 0.80
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    ),
    planta_maceta_pose=Pose(
        position=Point(x=0.85, y=0.00, z=0.84),
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    ),

    # ── Row 3  (y=-0.30) ─────────────────────────────────────
    esponja_lavaplatos_pose=Pose(
        position=Point(x=0.45, y=-0.30, z=0.84),
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    ),
    papas_fritas_pose=Pose(
        position=Point(x=0.65, y=-0.30, z=0.84),
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    ),
):
    model_path = "/home/masterchief/.gazebo/models"

    with open(os.path.join(model_path, "cafe_table", "model.sdf"), "r") as f:
        table_xml = f.read().replace("\n", "")
        
    with open(os.path.join(model_path, "block", "model.urdf"), "r") as f:
        block_xml = f.read().replace("\n", "")
        
    with open(os.path.join(model_path, "mustard", "model.sdf"), "r") as f:
        mustard_xml = f.read().replace("\n", "")
        
    with open(os.path.join(model_path, "strawberry", "strawberry.sdf"), "r") as f:
        strawberry_xml = f.read().replace("\n", "")
        
    with open(os.path.join(model_path, "banana", "banana.sdf"), "r") as f:
        banana_xml = f.read().replace("\n", "")
        
    with open(os.path.join(model_path, "coke_can", "model.sdf"), "r") as f:
        coke_can_xml = f.read().replace("\n", "")
        
    with open(os.path.join(model_path, "bowl", "model.sdf"), "r") as f:
        bowl_xml = f.read().replace("\n", "")
        
    with open(os.path.join(model_path, "planta_maceta", "model.sdf"), "r") as f:
        planta_maceta_xml = f.read().replace("\n", "")
        
    with open(os.path.join(model_path, "esponja_lavaplatos", "model.sdf"), "r") as f:
        esponja_lavaplatos_xml = f.read().replace("\n", "")
        
    with open(os.path.join(model_path, "papas_fritas", "model.sdf"), "r") as f:
        papas_fritas_xml = f.read().replace("\n", "")

    for name in ["cafe_table", "block", "mustard",
                 "strawberry", "banana", "coke_can",
                 "bowl", "planta_maceta","esponja_lavaplatos","papas_fritas"]:
        delete_if_exists(name)

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    rospy.loginfo("Spawning cafe_table...")
    spawn_sdf("cafe_table", table_xml, "/", table_pose, "world")

    # Row 1
    rospy.loginfo("Spawning mustard...")
    spawn_sdf("mustard", mustard_xml, "/", mustard_pose, "world")
    rospy.loginfo("Spawning coke_can...")
    spawn_sdf("coke_can", coke_can_xml, "/", coke_can_pose, "world")
    rospy.loginfo("Spawning bowl...")
    spawn_sdf("bowl", bowl_xml, "/", bowl_pose, "world")

    # Row 2
    rospy.loginfo("Spawning banana...")
    spawn_sdf("banana", banana_xml, "/", banana_pose, "world")
    rospy.loginfo("Spawning strawberry...")
    spawn_sdf("strawberry", strawberry_xml, "/", strawberry_pose, "world")
    rospy.loginfo("Spawning planta_maceta...")
    spawn_sdf("planta_maceta", planta_maceta_xml, "/", planta_maceta_pose, "world")

    # Row 3
    rospy.loginfo("Spawning esponja_lavaplatos...")
    spawn_sdf("esponja_lavaplatos", esponja_lavaplatos_xml, "/", esponja_lavaplatos_pose, "world")
    rospy.loginfo("Spawning papas_fritas...")
    spawn_sdf("papas_fritas", papas_fritas_xml, "/", papas_fritas_pose, "world")

    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    rospy.loginfo("Spawning block...")
    spawn_urdf("block", block_xml, "/", block_pose, "world")

    rospy.loginfo("Scene ready.")

if __name__ == "__main__":
    rospy.init_node("spawn_pick_place_scene")
    load_gazebo_models()
