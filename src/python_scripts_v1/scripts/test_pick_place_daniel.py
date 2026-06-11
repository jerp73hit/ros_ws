#!/usr/bin/env python3

import sys
import subprocess
import rospy
import math


from llm_api import (
    init_api,
    make_scene,
    adjust_grasp_waypoints,
    execute_waypoints,
    go_to_safe_pose,
)


# ──────────────────────────────────────────────────────────────
# CONFIGURACIÓN GENERAL
# ──────────────────────────────────────────────────────────────

DEFAULT_OBJECT = "banana"

# Este archivo ya tiene la pose calibrada de cámara.
# Si se quiere cambiar la pose, se edita SOLO ese archivo.
CAMERA_POSE_SCRIPT = "go_to_top_hand_camera_pos.py"

APPROACH_ORI = [180, 0, 0]

DEFAULT_TABLE_Z = 0.755
CAMERA_SETTLE_SECONDS = 1.0


# ──────────────────────────────────────────────────────────────
# AJUSTES LOCALES DE CENTRADO
# ──────────────────────────────────────────────────────────────
#
# Correcciones pequeñas sobre el centro detectado.
# Esto NO modifica frame2world.py ni llm_api.py.
#
# Según tu debug:
# - sponge se va un poco hacia Y negativo, entonces corregimos hacia +Y.
# - coke_can necesita un poco más hacia X positivo.

PICK_XY_OFFSETS = {
    "sponge":   {"x": 0.000, "y": 0.015},
    "coke_can": {"x": 0.012, "y": 0.000},
}


# ──────────────────────────────────────────────────────────────
# PERFILES DE AGARRE
# ──────────────────────────────────────────────────────────────
#
# grasp_height_ratio:
#   0.00 = cerca de la base
#   0.50 = centro vertical
#   1.00 = parte superior
#
# yaw:
#   fallback si el objeto no trae orientation desde frame2world.py.
#
# Para coke_can:
#   Se baja más el agarre para que no la tome tan arriba y no se caiga.

GRASP_PROFILES = {
    "block": {
        "grasp_height_ratio": 0.50,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 0.0,
        "min_tip_clearance": 0.010,
        "use_detected_orientation": False,
    },

    "sponge": {
        "grasp_height_ratio": 0.45,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 0.0,
        "min_tip_clearance": 0.008,
        "use_detected_orientation": True,
    },

    "potatoes": {
        "grasp_height_ratio": 0.50,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 0.0,
        "min_tip_clearance": 0.010,
        "use_detected_orientation": False,
    },

    "strawberry": {
        "grasp_height_ratio": 0.20,
        "z_offset": 0.000,
        "hover_height": 0.12,
        "yaw": 0.0,
        "min_tip_clearance": 0.004,
        "use_detected_orientation": False,
    },

    "coke_can": {
        "grasp_height_ratio": 0.30,
        "z_offset": -0.006,
        "hover_height": 0.15,
        "yaw": 0.0,
        "min_tip_clearance": 0.010,
        "use_detected_orientation": False,
    },

    "mustard": {
        "grasp_height_ratio": 0.45,
        "z_offset": 0.000,
        "hover_height": 0.17,
        "yaw": 90.0,
        "min_tip_clearance": 0.010,
        "use_detected_orientation": True,
    },

    "banana": {
        "grasp_height_ratio": 0.45,
        "z_offset": 0.000,
        "hover_height": 0.12,
        "yaw": 90.0,
        "min_tip_clearance": 0.006,
        "use_detected_orientation": True,
    },

    "bowl": {
        "grasp_height_ratio": 0.65,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 0.0,
        "min_tip_clearance": 0.010,
        "use_detected_orientation": False,
    },

    "plant": {
        "grasp_height_ratio": 0.22,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 0.0,
        "min_tip_clearance": 0.010,
        "use_detected_orientation": False,
    },
}


# ──────────────────────────────────────────────────────────────
# UTILIDADES
# ──────────────────────────────────────────────────────────────

def get_profile(object_name):
    return GRASP_PROFILES.get(object_name, GRASP_PROFILES["block"])


def run_camera_pose_script():
    print("\nMoviendo a la pose calibrada de cámara...")

    try:
        subprocess.check_call(
            ["rosrun", "python_scripts_v1", CAMERA_POSE_SCRIPT],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
    except subprocess.CalledProcessError:
        print("ERROR: Falló el script de pose de cámara:", CAMERA_POSE_SCRIPT)
        return False

    rospy.sleep(CAMERA_SETTLE_SECONDS)
    return True


def capture_scene_from_camera():
    if not run_camera_pose_script():
        return None

    scene = make_scene()

    if not scene or not scene.get("objects"):
        print("\nNo se detectaron objetos.")
        return None

    print("\nObjetos encontrados:")
    for name in scene["objects"].keys():
        print(" -", name)

    go_to_safe_pose()
    rospy.sleep(1.0)

    return scene


def ask_float(prompt, default_value=0.0):
    while True:
        value = input(prompt).strip()

        if value == "":
            return float(default_value)

        try:
            return float(value)
        except ValueError:
            print("Valor inválido. Escribe un número, por ejemplo: 0.15, -0.10 o 90")


def ask_object(scene, default_object):
    objects = scene.get("objects", {})

    while True:
        selected = input(
            "\nObjeto a mover [{}], 'r' para recapturar, o 'q' para salir: ".format(default_object)
        ).strip()

        if selected.lower() == "q":
            return "q"

        if selected.lower() == "r":
            return "r"

        if selected == "":
            selected = default_object

        if selected in objects:
            return selected

        print("'{}' no fue detectado.".format(selected))
        print("Disponibles:", list(objects.keys()))


def get_detected_yaw(scene, object_name):
    obj = scene["objects"][object_name]
    profile = get_profile(object_name)

    fallback_yaw = profile["yaw"]

    if not profile.get("use_detected_orientation", False):
        return fallback_yaw

    detected = obj.get("orientation", None)

    if detected is None:
        return fallback_yaw

    return float(detected)


def ask_yaw(auto_yaw):
    print("\nYaw del gripper:")
    print("  ENTER = usar yaw automático/detectado")
    print("  También puedes escribir 0, 90, -90, 180, etc.")

    value = input("Yaw final del gripper [{:.1f}]: ".format(auto_yaw)).strip()

    if value == "":
        return auto_yaw

    try:
        return float(value)
    except ValueError:
        print("Yaw inválido. Usando yaw automático:", auto_yaw)
        return auto_yaw


# ──────────────────────────────────────────────────────────────
# GEOMETRÍA DE AGARRE
# ──────────────────────────────────────────────────────────────

def apply_xy_offset(object_name, x, y):
    offset = PICK_XY_OFFSETS.get(object_name, {"x": 0.0, "y": 0.0})
    return x + offset["x"], y + offset["y"]


def snap_selected_object_xy(scene, object_name, yaw):
    obj = scene["objects"][object_name]
    center = obj["center"]

    probe = [
        {
            "pos": [center[0], center[1], center[2]],
            "ori": [180, 0, yaw],
            "gripper": "close",
        }
    ]

    snapped = adjust_grasp_waypoints(scene, probe)

    if not snapped:
        x, y = center[0], center[1]
    else:
        pos = snapped[0].get("pos", center)
        x, y = pos[0], pos[1]

    return apply_xy_offset(object_name, x, y)


def compute_grasp_z(scene, object_name):
    obj = scene["objects"][object_name]
    profile = get_profile(object_name)

    center = obj.get("center")
    base = obj.get("base")
    half = obj.get("half_extents")

    if center is None:
        raise RuntimeError("El objeto no tiene center: {}".format(object_name))

    if base is None or half is None:
        return center[2]

    base_z = base[2]
    half_z = half[2]
    full_height = 2.0 * half_z

    requested_z = (
        base_z
        + profile["grasp_height_ratio"] * full_height
        + profile["z_offset"]
    )

    table_z = scene.get("table_z", DEFAULT_TABLE_Z)
    min_z = table_z + profile["min_tip_clearance"]

    return max(requested_z, min_z)

def point_in_obb_xy(px, py, center, half_extents, orientation_deg, margin=0.005):
    """
    Checks if (px, py) is inside the XY footprint of an OBB (oriented bounding box).
    
    `orientation_deg` is the object's yaw in degrees (rotation around Z axis).
    We transform the point into the object's local frame and do an AABB check there.
    """
    # Translate point relative to object center
    dx = px - center[0]
    dy = py - center[1]

    # Rotate point into the object's local frame (inverse rotation = negative angle)
    angle_rad = math.radians(orientation_deg)
    cos_a = math.cos(-angle_rad)
    sin_a = math.sin(-angle_rad)

    local_x =  cos_a * dx - sin_a * dy
    local_y =  sin_a * dx + cos_a * dy

    # AABB check in local frame
    hx = half_extents[0] + margin
    hy = half_extents[1] + margin

    return abs(local_x) <= hx and abs(local_y) <= hy


def check_waypoints_collisions(scene, object_name, waypoints, exclude_names=None, margin=0.005):
    objects = scene.get("objects", {})
    warnings = []
    excluded = set(exclude_names or [])

    for i, wp in enumerate(waypoints):
        px, py = wp["pos"][0], wp["pos"][1]

        for name, obj in objects.items():
            if name == object_name:
                continue
            if name in excluded:
                continue

            center      = obj["center"]
            half        = obj["half_extents"]
            orientation = obj.get("orientation", 0.0)

            if point_in_obb_xy(px, py, center, half, orientation, margin=margin):
                warnings.append(
                    "Waypoint {} (gripper={}) XY=[{:.3f}, {:.3f}] "
                    "is inside OBB of '{}'  "
                    "center=[{:.3f}, {:.3f}] half=[{:.3f}, {:.3f}] ori={:.1f}°".format(
                        i, wp["gripper"],
                        px, py,
                        name,
                        center[0], center[1],
                        half[0], half[1],
                        orientation,
                    )
                )

    return warnings

def ask_excluded_objects(scene, object_name):
    """
    Ask the user which objects to exclude from collision checking.
    Returns a list of valid object names (never includes the target object).
    """
    available = [n for n in scene.get("objects", {}).keys() if n != object_name]

    if not available:
        return []

    print("\nObjetos disponibles para excluir de colisiones:", available)
    raw = input("Excluir objetos (separados por coma, ENTER = ninguno): ").strip()

    if not raw:
        return []

    excluded = []
    for token in raw.split(","):
        name = token.strip()
        if name in available:
            excluded.append(name)
        else:
            print("  '{}' no reconocido, ignorado.".format(name))

    return excluded

def build_waypoints(scene, object_name, dx, dy, yaw):
    profile = get_profile(object_name)

    hover_height = profile["hover_height"]

    pick_x, pick_y = snap_selected_object_xy(scene, object_name, yaw)
    grasp_z = compute_grasp_z(scene, object_name)

    place_x = pick_x + dx
    place_y = pick_y + dy

    pick_hover = [pick_x, pick_y, grasp_z + hover_height]
    pick_grasp = [pick_x, pick_y, grasp_z]

    place_hover = [place_x, place_y, grasp_z + hover_height]
    place_release = [place_x, place_y, grasp_z]

    return [
        {"pos": pick_hover, "ori": APPROACH_ORI, "gripper": "open"},
        {"pos": pick_hover, "ori": [180, 0, yaw], "gripper": "open"},
        {"pos": pick_grasp, "ori": [180, 0, yaw], "gripper": "close"},
        {"pos": pick_hover, "ori": [180, 0, yaw], "gripper": "close"},
        {"pos": place_hover, "ori": [180, 0, yaw], "gripper": "close"},
        {"pos": place_release, "ori": [180, 0, yaw], "gripper": "open"},
        {"pos": place_hover, "ori": [180, 0, yaw], "gripper": "open"},
    ]


def print_simple_waypoints(waypoints):
    print("\nWaypoints:")
    for i, wp in enumerate(waypoints):
        p = wp["pos"]
        print(
            "{}: pos=[{:.3f}, {:.3f}, {:.3f}], yaw={}, gripper={}".format(
                i,
                p[0],
                p[1],
                p[2],
                wp["ori"][2],
                wp["gripper"],
            )
        )


# ──────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────

def main():
    init_api()
    rospy.sleep(1.0)

    scene = capture_scene_from_camera()

    if scene is None:
        sys.exit(1)

    default_object = DEFAULT_OBJECT

    while not rospy.is_shutdown():
        selected_object = ask_object(scene, default_object)

        if selected_object == "q" or selected_object is None:
            print("Saliendo.")
            sys.exit(0)

        if selected_object == "r":
            new_scene = capture_scene_from_camera()
            if new_scene is not None:
                scene = new_scene
            continue

        default_object = selected_object

        auto_yaw = get_detected_yaw(scene, selected_object)

        print("\nObjeto seleccionado:", selected_object)
        print("Yaw automático:", round(auto_yaw, 2))

        dx  = ask_float("Mover en X [0.0]: ", 0.0)
        dy  = ask_float("Mover en Y [0.15]: ", 0.15)
        yaw = ask_yaw(auto_yaw)

        # ── Ask for collision exclusions (resets each iteration) ──
        excluded_objects = ask_excluded_objects(scene, selected_object)
        if excluded_objects:
            print("Excluyendo de colisiones:", excluded_objects)
        # ─────────────────────────────────────────────────────────

        waypoints = build_waypoints(
            scene=scene,
            object_name=selected_object,
            dx=dx,
            dy=dy,
            yaw=yaw,
        )

        print_simple_waypoints(waypoints)

        # ── Collision check ───────────────────────────────────────
        collision_warnings = check_waypoints_collisions(
            scene, selected_object, waypoints,
            exclude_names=excluded_objects,
            margin=0.005,
        )
        if collision_warnings:
            print("\n⚠  COLLISION WARNINGS:")
            for w in collision_warnings:
                print("  •", w)
            confirm = input(
                "\nHay colisiones potenciales. ¿Ejecutar de todas formas? "
                "[ENTER = sí, q = cancelar]: "
            ).strip().lower()
        else:
            print("\n✓ Sin colisiones XY detectadas.")
            confirm = input("\nEjecutar movimiento? [ENTER = sí, q = no]: ").strip().lower()
        # ─────────────────────────────────────────────────────────

        if confirm == "q":
            print("Movimiento cancelado.")
            continue  # excluded_objects goes out of scope, resets next iteration

        execute_waypoints(waypoints)

        print("\nMovimiento terminado.")
        print("Escribe 'r' para recapturar la escena si algún objeto se movió.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
