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

OBJECT_NAME = "block"

CAMERA_POSE_SCRIPT = "go_to_top_hand_camera_pos.py"

APPROACH_ORI = [180, 0, 0]

DEFAULT_TABLE_Z = 0.755
CAMERA_SETTLE_SECONDS = 1.0

# Movimiento alternado del cubo
MOVE_DX = 0.0
MOVE_DY = 0.15

# Tiempo entre ciclos
WAIT_BETWEEN_CYCLES = 1.0

# Si falla varias veces seguidas, se detiene solo
MAX_CONSECUTIVE_FAILURES = 3


# ──────────────────────────────────────────────────────────────
# AJUSTES LOCALES
# ──────────────────────────────────────────────────────────────

PICK_XY_OFFSETS = {
    "block": {"x": 0.000, "y": 0.000},
}


GRASP_PROFILES = {
    "block": {
        "grasp_height_ratio": 0.50,
        "z_offset": 0.000,
        "hover_height": 0.20,
        "yaw": 0.0,
        "min_tip_clearance": 0.010,
        "use_detected_orientation": True,
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
        print("No se detectaron objetos.")
        return None

    print("\nObjetos encontrados:")
    for name in scene["objects"].keys():
        print(" -", name)

    go_to_safe_pose()
    rospy.sleep(1.0)

    return scene


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def get_detected_yaw(scene, object_name):
    obj = scene["objects"][object_name]
    profile = get_profile(object_name)

    fallback_yaw = profile["yaw"]

    if not profile.get("use_detected_orientation", False):
        return fallback_yaw

    detected = obj.get("orientation", None)

    if detected is None:
        return fallback_yaw

    return normalize_angle_deg(float(detected))


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


def print_waypoints(waypoints):
    print("\nWaypoints generados:")
    for i, wp in enumerate(waypoints):
        p = wp["pos"]
        print(
            "{}: pos=[{:.3f}, {:.3f}, {:.3f}], yaw={:.2f}, gripper={}".format(
                i,
                p[0],
                p[1],
                p[2],
                wp["ori"][2],
                wp["gripper"],
            )
        )


# ──────────────────────────────────────────────────────────────
# LOOP AUTOMÁTICO
# ──────────────────────────────────────────────────────────────

def main():
    init_api()
    rospy.sleep(1.0)

    cycle = 0
    direction = 1.0
    failures = 0

    print("\nIniciando loop automático para recoger el cubo.")
    print("Objeto:", OBJECT_NAME)
    print("Presiona Ctrl+C para detener.\n")

    while not rospy.is_shutdown():
        cycle += 1

        print("\n" + "=" * 60)
        print("CICLO", cycle)
        print("=" * 60)

        scene = capture_scene_from_camera()

        if scene is None:
            failures += 1
            print("Fallo capturando escena. Fallos consecutivos:", failures)

            if failures >= MAX_CONSECUTIVE_FAILURES:
                print("Demasiados fallos consecutivos. Deteniendo.")
                break

            rospy.sleep(WAIT_BETWEEN_CYCLES)
            continue

        if OBJECT_NAME not in scene.get("objects", {}):
            failures += 1
            print("No se detectó '{}'. Fallos consecutivos: {}".format(
                OBJECT_NAME,
                failures,
            ))

            if failures >= MAX_CONSECUTIVE_FAILURES:
                print("No se pudo detectar el cubo varias veces. Deteniendo.")
                break

            rospy.sleep(WAIT_BETWEEN_CYCLES)
            continue

        failures = 0

        yaw = get_detected_yaw(scene, OBJECT_NAME)

        dx = MOVE_DX
        dy = direction * MOVE_DY

        print("\nObjeto seleccionado:", OBJECT_NAME)
        print("Yaw automático:", round(yaw, 2))
        print("Movimiento: dx={:.3f}, dy={:.3f}".format(dx, dy))

        waypoints = build_waypoints(
            scene=scene,
            object_name=OBJECT_NAME,
            dx=dx,
            dy=dy,
            yaw=yaw,
        )

        print_waypoints(waypoints)

        try:
            execute_waypoints(waypoints)
        except Exception as e:
            failures += 1
            print("ERROR ejecutando waypoints:", e)
            print("Fallos consecutivos:", failures)

            if failures >= MAX_CONSECUTIVE_FAILURES:
                print("Demasiados fallos ejecutando. Deteniendo.")
                break

            rospy.sleep(WAIT_BETWEEN_CYCLES)
            continue

        print("\nCiclo terminado correctamente.")

        # Cambia dirección para que el cubo vaya y vuelva.
        direction *= -1.0

        rospy.sleep(WAIT_BETWEEN_CYCLES)

    print("\nLoop terminado.")
    go_to_safe_pose()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nInterrumpido por usuario.")
        try:
            go_to_safe_pose()
        except Exception:
            pass