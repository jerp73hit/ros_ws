#!/usr/bin/env python3

import os
import math
import rospy

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


MODEL_PATH = os.path.expanduser("~/ros_ws_team/models")


def rpy_deg_to_quat(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Converts roll, pitch, yaw in degrees to geometry_msgs/Quaternion.
    """
    q = quaternion_from_euler(
        math.radians(roll),
        math.radians(pitch),
        math.radians(yaw)
    )

    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def make_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """
    Creates a Gazebo pose using Cartesian position and RPY orientation in degrees.
    """
    return Pose(
        position=Point(x=x, y=y, z=z),
        orientation=rpy_deg_to_quat(roll, pitch, yaw)
    )


def read_model(relative_path):
    path = os.path.join(MODEL_PATH, relative_path)

    if not os.path.exists(path):
        raise FileNotFoundError("Model file not found: {}".format(path))

    with open(path, "r") as f:
        return f.read().replace("\n", "")


def delete_if_exists(model_name):
    try:
        rospy.wait_for_service("/gazebo/delete_model", timeout=2.0)
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model(model_name)
    except Exception:
        pass


def spawn_sdf_model(spawn_sdf, name, xml, pose):
    rospy.loginfo("Spawning %s...", name)
    result = spawn_sdf(name, xml, "/", pose, "world")
    rospy.loginfo("%s spawn result: %s", name, result.status_message)


def spawn_urdf_model(spawn_urdf, name, xml, pose):
    rospy.loginfo("Spawning %s...", name)
    result = spawn_urdf(name, xml, "/", pose, "world")
    rospy.loginfo("%s spawn result: %s", name, result.status_message)


def load_gazebo_models():
    rospy.loginfo("Using model path: %s", MODEL_PATH)

    # -------------------------------------------------------------------------
    # Poses:
    # Position is Cartesian: x, y, z.
    # Orientation is intuitive: roll, pitch, yaw in degrees.
    # -------------------------------------------------------------------------

    table_pose = make_pose(
        x=0.75, y=0.00, z=0.00,
        roll=0, pitch=0, yaw=0
    )

    # Primary target
    block_pose = make_pose(
        x=0.55, y=-0.25, z=0.82,
        roll=0, pitch=0, yaw=0
    )

    # Row 1
    mustard_pose = make_pose(
        x=0.45, y=0.20, z=0.80,
        roll=0, pitch=0, yaw=0
    )

    coke_can_pose = make_pose(
        x=0.65, y=0.30, z=0.86,
        roll=0, pitch=0, yaw=0
    )

    bowl_pose = make_pose(
        x=0.85, y=0.20, z=0.84,
        roll=0, pitch=0, yaw=0
    )

    # Row 2
    banana_pose = make_pose(
        x=0.45, y=0.00, z=0.80,
        roll=0, pitch=0, yaw=90
    )

    strawberry_pose = make_pose(
        x=0.65, y=0.00, z=0.79,
        roll=0, pitch=0, yaw=0
    )

    planta_maceta_pose = make_pose(
        x=0.85, y=0.00, z=0.84,
        roll=0, pitch=0, yaw=0
    )

    # Row 3
    esponja_lavaplatos_pose = make_pose(
        x=0.45, y=-0.30, z=0.84,
        roll=0, pitch=0, yaw=0
    )

    papas_fritas_pose = make_pose(
        x=0.65, y=-0.30, z=0.84,
        roll=0, pitch=0, yaw=0
    )

    # -------------------------------------------------------------------------
    # Load XML models
    # -------------------------------------------------------------------------

    table_xml = read_model("cafe_table/model.sdf")
    block_xml = read_model("block/model.urdf")

    mustard_xml = read_model("mustard/model.sdf")
    strawberry_xml = read_model("strawberry/strawberry.sdf")
    banana_xml = read_model("banana/banana.sdf")
    coke_can_xml = read_model("coke_can/model.sdf")
    bowl_xml = read_model("bowl/model.sdf")
    planta_maceta_xml = read_model("planta_maceta/model.sdf")
    esponja_lavaplatos_xml = read_model("esponja_lavaplatos/model.sdf")
    papas_fritas_xml = read_model("papas_fritas/model.sdf")

    # -------------------------------------------------------------------------
    # Delete previous models
    # -------------------------------------------------------------------------

    model_names = [
        "cafe_table",
        "block",
        "mustard",
        "strawberry",
        "banana",
        "coke_can",
        "bowl",
        "planta_maceta",
        "esponja_lavaplatos",
        "papas_fritas",
    ]

    for name in model_names:
        delete_if_exists(name)

    # -------------------------------------------------------------------------
    # Spawn models
    # -------------------------------------------------------------------------

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    spawn_sdf_model(spawn_sdf, "cafe_table", table_xml, table_pose)

    spawn_sdf_model(spawn_sdf, "mustard", mustard_xml, mustard_pose)
    spawn_sdf_model(spawn_sdf, "coke_can", coke_can_xml, coke_can_pose)
    spawn_sdf_model(spawn_sdf, "bowl", bowl_xml, bowl_pose)

    spawn_sdf_model(spawn_sdf, "banana", banana_xml, banana_pose)
    spawn_sdf_model(spawn_sdf, "strawberry", strawberry_xml, strawberry_pose)
    spawn_sdf_model(spawn_sdf, "planta_maceta", planta_maceta_xml, planta_maceta_pose)

    spawn_sdf_model(spawn_sdf, "esponja_lavaplatos", esponja_lavaplatos_xml, esponja_lavaplatos_pose)
    spawn_sdf_model(spawn_sdf, "papas_fritas", papas_fritas_xml, papas_fritas_pose)

    spawn_urdf_model(spawn_urdf, "block", block_xml, block_pose)

    rospy.loginfo("Scene ready.")


if __name__ == "__main__":
    rospy.init_node("spawn_pick_place_scene")
    load_gazebo_models()
