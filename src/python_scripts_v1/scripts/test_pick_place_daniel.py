#!/usr/bin/env python3

import sys
import rospy

from llm_api import (
    init_api,
    make_scene,
    adjust_grasp_waypoints,
    execute_waypoints,
    get_limb,
    go_to_safe_pose,
)


# ──────────────────────────────────────────────────────────────
# CONFIGURACIÓN GENERAL
# ──────────────────────────────────────────────────────────────

DEFAULT_OBJECT = "strawberry"

APPROACH_ORI = [180, 0, 0]

SPEED = 0.12
TIMEOUT = 25.0

DEFAULT_TABLE_Z = 0.755


# ──────────────────────────────────────────────────────────────
# PERFILES LOCALES DE AGARRE
# ──────────────────────────────────────────────────────────────
#
# grasp_height_ratio:
#   0.00 = cerca de la base del objeto
#   0.50 = centro vertical del objeto
#   1.00 = parte superior del objeto
#
# Para strawberry usamos 0.20 porque la pinza debe bajar más que el
# punto central que calcula adjust_grasp_waypoints().
#
# min_tip_clearance:
#   Distancia mínima permitida entre right_gripper_tip y la mesa.
#
# hover_height:
#   Altura interna de aproximación. No se solicita en terminal.

GRASP_PROFILES = {
    "block": {
        "grasp_height_ratio": 0.50,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 180.0,
        "min_tip_clearance": 0.010,
    },

    "sponge": {
        "grasp_height_ratio": 0.45,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 180.0,
        "min_tip_clearance": 0.008,
    },

    "potatoes": {
        "grasp_height_ratio": 0.50,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 180.0,
        "min_tip_clearance": 0.010,
    },

    "strawberry": {
        "grasp_height_ratio": 0.20,
        "z_offset": 0.000,
        "hover_height": 0.12,
        "yaw": 180.0,
        "min_tip_clearance": 0.004,
    },

    "coke_can": {
        "grasp_height_ratio": 0.50,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 180.0,
        "min_tip_clearance": 0.010,
    },

    "mustard": {
        "grasp_height_ratio": 0.55,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 180.0,
        "min_tip_clearance": 0.010,
    },

    "banana": {
        "grasp_height_ratio": 0.45,
        "z_offset": 0.000,
        "hover_height": 0.12,
        "yaw": 180.0,
        "min_tip_clearance": 0.006,
    },

    "bowl": {
        "grasp_height_ratio": 0.65,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 180.0,
        "min_tip_clearance": 0.010,
    },

    "plant": {
        # La visión detecta la planta completa, pero queremos agarrar
        # la maceta en la parte inferior, no las hojas.
        "grasp_height_ratio": 0.22,
        "z_offset": 0.000,
        "hover_height": 0.15,
        "yaw": 180.0,
        "min_tip_clearance": 0.010,
    },
}


# ──────────────────────────────────────────────────────────────
# POSE SUPERIOR DE CÁMARA
# ──────────────────────────────────────────────────────────────

TOP_CAMERA_POSE = {
    "right_j0": -1.1407884434852917,
    "right_j1": -0.8546896185262485,
    "right_j2": -1.9019301841097436,
    "right_j3": -1.2891338207310437,
    "right_j4":  2.488137281192378,
    "right_j5":  0.008300765583789449,
    "right_j6":  1.7659537569669599,
}


def get_profile(object_name):
    if object_name in GRASP_PROFILES:
        return GRASP_PROFILES[object_name]

    rospy.logwarn(
        "No local grasp profile for '%s'. Using block profile.",
        object_name,
    )
    return GRASP_PROFILES["block"]


def move_to_top_camera_pose():
    limb = get_limb()

    if limb is None:
        rospy.logerr("No limb available. Did you call init_api() first?")
        return False

    rospy.loginfo("Moving to top camera analysis pose...")
    limb.set_joint_position_speed(SPEED)
    limb.move_to_joint_positions(TOP_CAMERA_POSE, timeout=TIMEOUT)
    rospy.sleep(1.0)

    rospy.loginfo("Top camera pose reached.")
    return True


# ──────────────────────────────────────────────────────────────
# ENTRADAS DE TERMINAL
# ──────────────────────────────────────────────────────────────

def ask_float(prompt, default_value=0.0):
    while True:
        value = input(prompt).strip()

        if value == "":
            return float(default_value)

        try:
            return float(value)
        except ValueError:
            print("Valor inválido. Escribe un número, por ejemplo: 0.15, -0.10 o 0")


def ask_object(scene, default_object=DEFAULT_OBJECT):
    objects = scene.get("objects", {})

    if not objects:
        print("\nERROR: No se detectaron objetos en la escena.")
        return None

    print("\n========== OBJETOS DETECTADOS ==========")

    for name, obj in objects.items():
        center = obj.get("center")
        base = obj.get("base")
        half = obj.get("half_extents")
        confidence = obj.get("confidence")

        print(" - {}".format(name))

        if center is not None:
            print(
                "   center = [{:.3f}, {:.3f}, {:.3f}]".format(
                    center[0], center[1], center[2]
                )
            )

        if base is not None:
            print(
                "   base   = [{:.3f}, {:.3f}, {:.3f}]".format(
                    base[0], base[1], base[2]
                )
            )

        if half is not None:
            print(
                "   half   = [{:.3f}, {:.3f}, {:.3f}]".format(
                    half[0], half[1], half[2]
                )
            )

        if confidence is not None:
            print("   conf   = {:.2f}".format(confidence))

    print("========================================")

    while True:
        selected = input(
            "Objeto a mover [{}] o 'q' para salir: ".format(default_object)
        ).strip()

        if selected.lower() == "q":
            return None

        if selected == "":
            selected = default_object

        if selected in objects:
            return selected

        print("\n'{}' no está en la escena detectada.".format(selected))
        print("Opciones disponibles:", list(objects.keys()))


# ──────────────────────────────────────────────────────────────
# GEOMETRÍA Y PUNTO DE AGARRE
# ──────────────────────────────────────────────────────────────

def snap_selected_object_xy(scene, object_name, yaw):
    """
    Usa adjust_grasp_waypoints() solamente con un waypoint.

    De esta forma aprovechamos el ajuste X/Y del código compartido,
    pero evitamos que modifique otros waypoints de traslado que también
    tienen el gripper cerrado.
    """

    obj = scene["objects"][object_name]
    center = obj["center"]

    probe_waypoint = [
        {
            "pos": [center[0], center[1], center[2]],
            "ori": [180, 0, yaw],
            "gripper": "close",
        }
    ]

    snapped = adjust_grasp_waypoints(scene, probe_waypoint)

    if not snapped:
        return center[0], center[1]

    snapped_pos = snapped[0].get("pos", center)

    return snapped_pos[0], snapped_pos[1]


def compute_grasp_z(scene, object_name):
    """
    Calcula un punto de agarre dentro de la altura del objeto.

    No cambia la geometría de colisión.
    No usa siempre el centro vertical.

    Para strawberry:
      base_z + 20 % de la altura detectada
    """

    obj = scene["objects"][object_name]
    profile = get_profile(object_name)

    base = obj.get("base")
    half = obj.get("half_extents")
    center = obj.get("center")

    if base is None or half is None:
        rospy.logwarn(
            "Missing base/half_extents for %s. Using detected center z.",
            object_name,
        )
        return center[2]

    base_z = base[2]
    half_z = half[2]
    full_height = 2.0 * half_z

    ratio = profile["grasp_height_ratio"]
    z_offset = profile["z_offset"]

    requested_grasp_z = base_z + ratio * full_height + z_offset

    table_z = scene.get("table_z", DEFAULT_TABLE_Z)
    min_grasp_z = table_z + profile["min_tip_clearance"]

    grasp_z = max(requested_grasp_z, min_grasp_z)

    rospy.loginfo("========== LOCAL GRASP GEOMETRY ==========")
    rospy.loginfo("Object: %s", object_name)
    rospy.loginfo("Detected center_z: %.4f", center[2])
    rospy.loginfo("Detected base_z:   %.4f", base_z)
    rospy.loginfo("Detected half_z:   %.4f", half_z)
    rospy.loginfo("Detected height:   %.4f", full_height)
    rospy.loginfo("Grasp ratio:       %.2f", ratio)
    rospy.loginfo("Requested grasp_z: %.4f", requested_grasp_z)
    rospy.loginfo("Minimum grasp_z:   %.4f", min_grasp_z)
    rospy.loginfo("Final grasp_z:     %.4f", grasp_z)
    rospy.loginfo("==========================================")

    return grasp_z


def build_waypoints(scene, object_name, dx, dy):
    obj = scene["objects"][object_name]
    profile = get_profile(object_name)

    yaw = profile["yaw"]
    hover_height = profile["hover_height"]

    pick_x, pick_y = snap_selected_object_xy(scene, object_name, yaw)
    grasp_z = compute_grasp_z(scene, object_name)

    place_x = pick_x + dx
    place_y = pick_y + dy

    pick_hover = [pick_x, pick_y, grasp_z + hover_height]
    pick_grasp = [pick_x, pick_y, grasp_z]

    place_hover = [place_x, place_y, grasp_z + hover_height]
    place_release = [place_x, place_y, grasp_z]

    waypoints = [
        {
            "pos": pick_hover,
            "ori": APPROACH_ORI,
            "gripper": "open",
        },
        {
            "pos": pick_hover,
            "ori": [180, 0, yaw],
            "gripper": "open",
        },
        {
            "pos": pick_grasp,
            "ori": [180, 0, yaw],
            "gripper": "close",
        },
        {
            "pos": pick_hover,
            "ori": [180, 0, yaw],
            "gripper": "close",
        },
        {
            "pos": place_hover,
            "ori": [180, 0, yaw],
            "gripper": "close",
        },
        {
            "pos": place_release,
            "ori": [180, 0, yaw],
            "gripper": "open",
        },
        {
            "pos": place_hover,
            "ori": [180, 0, yaw],
            "gripper": "open",
        },
    ]

    return waypoints


def print_waypoints(waypoints):
    print("\n========== FINAL WAYPOINTS ==========")

    for index, waypoint in enumerate(waypoints):
        pos = waypoint["pos"]
        print(
            "[{}] pos=[{:.3f}, {:.3f}, {:.3f}] ori={} gripper={}".format(
                index,
                pos[0],
                pos[1],
                pos[2],
                waypoint["ori"],
                waypoint["gripper"],
            )
        )

    print("=====================================")


# ──────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────

def main():
    init_api()
    rospy.sleep(1.0)

    if not move_to_top_camera_pose():
        print("No se pudo mover a la pose de cámara.")
        sys.exit(1)

    rospy.loginfo("Capturing scene from top camera pose...")
    scene = make_scene()

    if not scene or not scene.get("objects"):
        print("No se pudo construir una escena válida.")
        sys.exit(1)

    rospy.loginfo("Scene captured. Moving to safe pose before pick/place...")
    go_to_safe_pose()
    rospy.sleep(1.0)

    selected_object = ask_object(scene)

    if selected_object is None:
        print("Saliendo.")
        sys.exit(0)

    print("\nObjeto seleccionado:", selected_object)

    print("\nIndica cuánto quieres mover el objeto en metros.")
    dx = ask_float("Mover en X [0.0]: ", 0.0)
    dy = ask_float("Mover en Y [0.15]: ", 0.15)

    waypoints = build_waypoints(
        scene=scene,
        object_name=selected_object,
        dx=dx,
        dy=dy,
    )

    print_waypoints(waypoints)

    profile = get_profile(selected_object)

    print("\nResumen:")
    print("  Objeto:", selected_object)
    print("  dx:", dx)
    print("  dy:", dy)
    print("  grasp height ratio:", profile["grasp_height_ratio"])
    print("  hover interno:", profile["hover_height"])
    print("  yaw:", profile["yaw"])

    confirm = input("\nEjecutar movimiento? [ENTER = sí, q = no]: ").strip().lower()

    if confirm == "q":
        print("Movimiento cancelado.")
        sys.exit(0)

    execute_waypoints(waypoints)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
