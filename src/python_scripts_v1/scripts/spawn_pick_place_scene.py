#!/usr/bin/env python3

import os
import math
from pathlib import Path

import rospy
import rospkg

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


PACKAGE_NAME = "python_scripts_v1"


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


def _candidate_has_required_models(model_path):
    """
    Basic validation to avoid selecting the wrong folder.
    """
    model_path = Path(model_path)

    required_files = [
        model_path / "cafe_table" / "model.sdf",
        model_path / "block" / "model.urdf",
    ]

    return all(path.exists() for path in required_files)


def get_model_path():
    """
    Locate the shared models folder in a workspace-independent way.

    Priority:
        1. PICK_PLACE_MODELS_PATH environment variable.
        2. Any valid folder in GAZEBO_MODEL_PATH.
        3. Search upward from the ROS package path until finding <repo_root>/models.

    Expected repository layout:
        <repo_root>/models
        <repo_root>/src/python_scripts_v1

    This avoids hardcoding paths such as ~/ros_ws_team/models.
    """

    # ---------------------------------------------------------------------
    # 1. Explicit override, useful for teammates with unusual layouts.
    # ---------------------------------------------------------------------
    env_override = os.environ.get("PICK_PLACE_MODELS_PATH", "").strip()

    if env_override:
        candidate = Path(env_override).expanduser().resolve()

        if _candidate_has_required_models(candidate):
            return str(candidate)

        raise FileNotFoundError(
            "PICK_PLACE_MODELS_PATH is set, but it does not contain the "
            "required models.\n"
            "PICK_PLACE_MODELS_PATH={}".format(candidate)
        )

    # ---------------------------------------------------------------------
    # 2. Try GAZEBO_MODEL_PATH.
    # ---------------------------------------------------------------------
    gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")

    for raw_candidate in gazebo_model_path.split(":"):
        if not raw_candidate.strip():
            continue

        candidate = Path(raw_candidate).expanduser().resolve()

        if _candidate_has_required_models(candidate):
            return str(candidate)

    # ---------------------------------------------------------------------
    # 3. Search upward from the ROS package path.
    # ---------------------------------------------------------------------
    try:
        pkg_path = Path(rospkg.RosPack().get_path(PACKAGE_NAME)).resolve()
    except rospkg.ResourceNotFound:
        raise RuntimeError(
            "Could not find ROS package '{}'. Did you source the correct "
            "workspace?".format(PACKAGE_NAME)
        )

    # Example:
    #   pkg_path = <repo_root>/src/python_scripts_v1
    #   possible parents:
    #       <repo_root>/src/python_scripts_v1
    #       <repo_root>/src
    #       <repo_root>
    #       ...
    search_roots = [pkg_path] + list(pkg_path.parents)

    for root in search_roots:
        candidate = root / "models"

        if _candidate_has_required_models(candidate):
            return str(candidate.resolve())

    # ---------------------------------------------------------------------
    # 4. Nothing worked.
    # ---------------------------------------------------------------------
    raise FileNotFoundError(
        "Models folder not found.\n\n"
        "Tried:\n"
        "  1. PICK_PLACE_MODELS_PATH\n"
        "  2. GAZEBO_MODEL_PATH\n"
        "  3. Searching upward from package path: {}\n\n"
        "Expected a folder containing at least:\n"
        "  models/cafe_table/model.sdf\n"
        "  models/block/model.urdf\n\n"
        "Recommended repository layout:\n"
        "  <repo_root>/models\n"
        "  <repo_root>/src/python_scripts_v1".format(pkg_path)
    )


MODEL_PATH = get_model_path()


def read_model(relative_path):
    path = Path(MODEL_PATH) / relative_path

    if not path.exists():
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
        x=0.55, y=-0.25, z=0.795,
        roll=0, pitch=0, yaw=0
    )

    # Row 1
    mustard_pose = make_pose(
        x=0.5, y=0.20, z=0.79,
        roll=0, pitch=0, yaw=0
    )

    coke_can_pose = make_pose(
        x=0.65, y=0.30, z=0.78,
        roll=0, pitch=0, yaw=0
    )

    bowl_pose = make_pose(
        x=0.85, y=0.20, z=0.78,
        roll=0, pitch=0, yaw=0
    )

    # Row 2
    banana_pose = make_pose(
        x=0.45, y=0.00, z=0.80,
        roll=0, pitch=0, yaw=90
    )

    strawberry_pose = make_pose(
        x=0.65, y=0.00, z=0.78,
        roll=0, pitch=0, yaw=0
    )

    plant_pose = make_pose(
        x=0.85, y=0.00, z=0.78,
        roll=0, pitch=0, yaw=0
    )

    # Row 3
    sponge_pose = make_pose(
        x=0.45, y=-0.30, z=0.78,
        roll=0, pitch=0, yaw=0
    )

    potatoes_pose = make_pose(
        x=0.65, y=-0.30, z=0.78,
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
    plant_xml = read_model("plant/model.sdf")
    sponge_xml = read_model("sponge/model.sdf")
    potatoes_xml = read_model("potatoes/model.sdf")

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
        "plant",
        "sponge",
        "potatoes",
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
    spawn_sdf_model(spawn_sdf, "plant", plant_xml, plant_pose)

    spawn_sdf_model(spawn_sdf, "sponge", sponge_xml, sponge_pose)
    spawn_sdf_model(spawn_sdf, "potatoes", potatoes_xml, potatoes_pose)

    spawn_urdf_model(spawn_urdf, "block", block_xml, block_pose)

    rospy.loginfo("Scene ready.")


if __name__ == "__main__":
    rospy.init_node("spawn_pick_place_scene")
    load_gazebo_models()