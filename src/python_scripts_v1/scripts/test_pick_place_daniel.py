#!/usr/bin/env python3

import sys
import rospy

from gazebo_msgs.srv import GetModelState

from llm_api import (
    init_api,
    make_scene,
    get_limb,
    get_gripper,
    move_to_pose,
    move_straight_z,
    go_to_safe_pose,
)


# ──────────────────────────────────────────────────────────────
# CONFIGURACIÓN GENERAL
# ──────────────────────────────────────────────────────────────

DEFAULT_OBJECT = "sponge"

HOVER_HEIGHT = 0.15

APPROACH_ORI = [180, 0, 0]
GRASP_ORI = [180, 0, 180]

SPEED = 0.12
TIMEOUT = 25.0

SAWYER_BASE_Z = 0.93
TABLE_Z = 0.755
MIN_Z_ABOVE_TABLE = 0.010

# Usar Gazebo para calcular el punto real de agarre.
# YOLO sigue siendo usado para detectar/listar objetos.
USE_GAZEBO_TRUTH_FOR_PICK = True


# ──────────────────────────────────────────────────────────────
# GEOMETRÍA REAL APROXIMADA DE MODELOS
# ──────────────────────────────────────────────────────────────
#
# half_z debe coincidir con la mitad de la altura física/collision box.
#
# potatoes:
#   collision size z = 0.08
#   half_z = 0.04
#   get_model_state z ≈ 0.774
#   center_z ≈ 0.774 + 0.04 = 0.814

GAZEBO_HALF_EXTENTS = {
    "potatoes": [0.040, 0.040, 0.040],
    "sponge": [0.0375, 0.02625, 0.01125],
    "block": [0.028, 0.028, 0.028],
    "coke_can": [0.033, 0.033, 0.058],
    "mustard": [0.038, 0.030, 0.075],
    "banana": [0.110, 0.030, 0.022],
    "strawberry": [0.025, 0.025, 0.022],
    "bowl": [0.095, 0.095, 0.028],
    "planta_maceta": [0.065, 0.065, 0.090],
}

# Offset adicional sobre el centro físico calculado.
#
# Para bajar más: valor negativo.
# Para subir más: valor positivo.
#
# potatoes:
#   centro físico ≈ 0.814
#   con -0.010 → grasp_z ≈ 0.804
# Esto debería meter la garra más dentro del volumen, sin irse a la mesa.
GRASP_Z_OFFSET_BY_OBJECT = {
    "potatoes": -0.010,
    "sponge": -0.002,
    "block": 0.000,
    "coke_can": 0.000,
    "mustard": 0.000,
    "banana": 0.000,
    "strawberry": 0.000,
    "bowl": 0.020,
    "planta_maceta": 0.020,
}

RELEASE_Z_OFFSET_BY_OBJECT = {
    "potatoes": 0.000,
    "sponge": 0.000,
    "block": 0.000,
    "coke_can": 0.000,
    "mustard": 0.000,
    "banana": 0.000,
    "strawberry": 0.000,
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
# INPUT
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
        center = obj.get("center", None)
        base = obj.get("base", None)
        half = obj.get("half_extents", None)
        conf = obj.get("confidence", None)

        print(" - {}".format(name))

        if center is not None:
            print("   vision center = [{:.3f}, {:.3f}, {:.3f}]".format(
                center[0], center[1], center[2]
            ))

        if base is not None:
            print("   vision base   = [{:.3f}, {:.3f}, {:.3f}]".format(
                base[0], base[1], base[2]
            ))

        if half is not None:
            print("   half extents  = [{:.3f}, {:.3f}, {:.3f}]".format(
                half[0], half[1], half[2]
            ))

        if conf is not None:
            print("   confidence    = {:.2f}".format(conf))

    print("========================================")

    while True:
        selected = input("Objeto a mover [{}] o 'q' para salir: ".format(default_object)).strip()

        if selected.lower() == "q":
            return None

        if selected == "":
            selected = default_object

        if selected in objects:
            return selected

        print("\n'{}' no está en la escena detectada.".format(selected))
        print("Opciones disponibles:", list(objects.keys()))


# ──────────────────────────────────────────────────────────────
# GEOMETRÍA DEL OBJETO
# ──────────────────────────────────────────────────────────────

def get_gazebo_model_pose(model_name):
    rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    resp = get_state(model_name, "world")

    if not resp.success:
        raise RuntimeError(resp.status_message)

    return resp.pose


def get_selected_object_geometry(scene, object_name):
    """
    Retorna:
      x, y, center_z, base_z, half_z

    Si USE_GAZEBO_TRUTH_FOR_PICK=True, usa Gazebo para el objeto seleccionado.
    Si no, usa la estimación de visión.
    """

    obj = scene["objects"][object_name]

    vision_center = obj["center"]
    vision_base = obj.get("base", None)
    vision_half = obj.get("half_extents", None)

    if not USE_GAZEBO_TRUTH_FOR_PICK:
        half_z = vision_half[2] if vision_half is not None else 0.03
        base_z = vision_base[2] if vision_base is not None else vision_center[2] - half_z
        return vision_center[0], vision_center[1], vision_center[2], base_z, half_z

    if object_name not in GAZEBO_HALF_EXTENTS:
        rospy.logwarn(
            "No Gazebo geometry configured for %s. Using vision geometry.",
            object_name,
        )
        half_z = vision_half[2] if vision_half is not None else 0.03
        base_z = vision_base[2] if vision_base is not None else vision_center[2] - half_z
        return vision_center[0], vision_center[1], vision_center[2], base_z, half_z

    pose = get_gazebo_model_pose(object_name)

    hx, hy, hz = GAZEBO_HALF_EXTENTS[object_name]

    gazebo_x = pose.position.x
    gazebo_y = pose.position.y
    gazebo_base_z = pose.position.z
    gazebo_center_z = gazebo_base_z + hz

    rospy.loginfo("========== DANIEL GEOMETRY DEBUG ==========")
    rospy.loginfo("Object: %s", object_name)

    rospy.loginfo(
        "Vision center: x=%.3f y=%.3f z=%.3f",
        vision_center[0], vision_center[1], vision_center[2],
    )

    if vision_base is not None:
        rospy.loginfo(
            "Vision base:   x=%.3f y=%.3f z=%.3f",
            vision_base[0], vision_base[1], vision_base[2],
        )

    if vision_half is not None:
        rospy.loginfo(
            "Vision half:   x=%.3f y=%.3f z=%.3f",
            vision_half[0], vision_half[1], vision_half[2],
        )

    rospy.loginfo(
        "Gazebo base:   x=%.3f y=%.3f z=%.3f",
        gazebo_x, gazebo_y, gazebo_base_z,
    )
    rospy.loginfo(
        "Gazebo center: x=%.3f y=%.3f z=%.3f",
        gazebo_x, gazebo_y, gazebo_center_z,
    )
    rospy.loginfo("===========================================")

    return gazebo_x, gazebo_y, gazebo_center_z, gazebo_base_z, hz


# ──────────────────────────────────────────────────────────────
# WAYPOINTS
# ──────────────────────────────────────────────────────────────

def build_waypoints(scene, object_name, dx, dy):
    x, y, center_z, base_z, half_z = get_selected_object_geometry(scene, object_name)

    table_z = scene.get("table_z", TABLE_Z)
    min_safe_z = table_z + MIN_Z_ABOVE_TABLE

    grasp_offset = GRASP_Z_OFFSET_BY_OBJECT.get(object_name, 0.0)
    release_offset = RELEASE_Z_OFFSET_BY_OBJECT.get(object_name, 0.0)

    grasp_z = center_z + grasp_offset
    release_z = center_z + release_offset

    if grasp_z < min_safe_z:
        rospy.logwarn(
            "Grasp z limited by table safety: requested %.3f -> %.3f",
            grasp_z,
            min_safe_z,
        )
        grasp_z = min_safe_z

    if release_z < min_safe_z:
        release_z = min_safe_z

    target_x = x + dx
    target_y = y + dy

    pick_hover = [x, y, grasp_z + HOVER_HEIGHT]
    pick_grasp = [x, y, grasp_z]

    place_hover = [target_x, target_y, release_z + HOVER_HEIGHT]
    place_release = [target_x, target_y, release_z]

    rospy.loginfo(
        "Daniel final geometry for %s: base_z=%.3f center_z=%.3f half_z=%.3f grasp_offset=%.3f grasp_z=%.3f release_z=%.3f",
        object_name,
        base_z,
        center_z,
        half_z,
        grasp_offset,
        grasp_z,
        release_z,
    )

    waypoints = [
        {"name": "pick_hover_approach", "pos": pick_hover,    "ori": APPROACH_ORI, "gripper": "open"},
        {"name": "pick_hover_align",    "pos": pick_hover,    "ori": GRASP_ORI,    "gripper": "open"},
        {"name": "pick_grasp",          "pos": pick_grasp,    "ori": GRASP_ORI,    "gripper": "close"},
        {"name": "pick_lift",           "pos": pick_hover,    "ori": GRASP_ORI,    "gripper": "close"},
        {"name": "place_hover",         "pos": place_hover,   "ori": GRASP_ORI,    "gripper": "close"},
        {"name": "place_release",       "pos": place_release, "ori": GRASP_ORI,    "gripper": "open"},
        {"name": "place_lift",          "pos": place_hover,   "ori": GRASP_ORI,    "gripper": "open"},
    ]

    return waypoints


def print_waypoints(title, waypoints):
    print("\n========== {} ==========".format(title))
    for i, wp in enumerate(waypoints):
        pos_txt = ["{:.3f}".format(v) for v in wp["pos"]]
        print("[{}] {} | pos={} ori={} gripper={}".format(
            i,
            wp["name"],
            pos_txt,
            wp["ori"],
            wp["gripper"],
        ))
    print("=" * (22 + len(title)))


# ──────────────────────────────────────────────────────────────
# EJECUCIÓN LOCAL
# ──────────────────────────────────────────────────────────────

def execute_waypoints_daniel(waypoints):
    limb = get_limb()
    gripper = get_gripper()

    if limb is None or gripper is None:
        rospy.logerr("Limb or gripper not available.")
        return False

    gripper.open()
    rospy.sleep(0.3)

    ep = limb.endpoint_pose()
    cur_x = ep["position"].x
    cur_y = ep["position"].y
    cur_z_w = ep["position"].z + SAWYER_BASE_Z

    rospy.loginfo(
        "Current endpoint world-like pose: x=%.3f y=%.3f z=%.3f",
        cur_x,
        cur_y,
        cur_z_w,
    )

    for i, wp in enumerate(waypoints):
        name = wp["name"]
        x, y, z_t = wp["pos"]
        r, p, ya = wp["ori"]

        rospy.loginfo(
            "Waypoint %d/%d [%s]: x=%.3f y=%.3f z=%.3f rpy=(%.1f, %.1f, %.1f) gripper=%s",
            i + 1,
            len(waypoints),
            name,
            x,
            y,
            z_t,
            r,
            p,
            ya,
            wp["gripper"],
        )

        xy_change = abs(x - cur_x) > 0.005 or abs(y - cur_y) > 0.005
        z_change = abs(z_t - cur_z_w) > 0.005

        if z_change:
            if xy_change:
                rospy.loginfo("Moving laterally at current height first...")
                ok = move_to_pose(x, y, cur_z_w, r, p, ya)

                if not ok:
                    rospy.logerr("Failed lateral move before waypoint [%s]. Aborting.", name)
                    return False

            rospy.loginfo("Moving vertically to target z...")
            ok = move_straight_z(x, y, cur_z_w, z_t, r, p, ya)

            if not ok:
                rospy.logerr("Failed vertical move at waypoint [%s]. Aborting.", name)
                return False

        else:
            ok = move_to_pose(x, y, z_t, r, p, ya)

            if not ok:
                rospy.logerr("Failed move_to_pose at waypoint [%s]. Aborting.", name)
                return False

        cur_x, cur_y, cur_z_w = x, y, z_t

        if wp["gripper"] == "open":
            gripper.open()
        elif wp["gripper"] == "close":
            gripper.close()

        rospy.sleep(0.4)

    rospy.loginfo("Daniel waypoint execution complete. Returning to safe pose.")
    go_to_safe_pose()

    return True


# ──────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────

def main():
    init_api()
    rospy.sleep(1.0)

    ok = move_to_top_camera_pose()
    if not ok:
        print("No se pudo mover a la pose de cámara.")
        sys.exit(1)

    rospy.loginfo("Capturing scene from top camera pose...")
    scene = make_scene()

    rospy.loginfo("Scene captured. Moving back to safe pose before pick/place...")
    go_to_safe_pose()
    rospy.sleep(1.0)

    selected_object = ask_object(scene, DEFAULT_OBJECT)

    if selected_object is None:
        print("Saliendo.")
        sys.exit(0)

    print("\nObjeto seleccionado:", selected_object)

    print("\nIndica cuánto quieres mover el objeto en metros.")
    print("Ejemplos:")
    print("  dx = 0.15   mueve +15 cm en X")
    print("  dx = -0.10  mueve -10 cm en X")
    print("  dy = 0.15   mueve +15 cm en Y")
    print("  dy = -0.15  mueve -15 cm en Y")

    dx = ask_float("Mover en X [0.0]: ", 0.0)
    dy = ask_float("Mover en Y [0.15]: ", 0.15)

    waypoints = build_waypoints(
        scene=scene,
        object_name=selected_object,
        dx=dx,
        dy=dy,
    )

    print_waypoints("DANIEL WAYPOINTS", waypoints)

    print("\nResumen:")
    print("  Objeto:", selected_object)
    print("  dx:", dx)
    print("  dy:", dy)
    print("  Gazebo truth:", USE_GAZEBO_TRUTH_FOR_PICK)
    print("  hover interno:", HOVER_HEIGHT)
    print("  grasp offset:", GRASP_Z_OFFSET_BY_OBJECT.get(selected_object, 0.0))

    confirm = input("\nEjecutar movimiento? [ENTER = sí, q = no]: ").strip().lower()

    if confirm == "q":
        print("Movimiento cancelado.")
        sys.exit(0)

    execute_waypoints_daniel(waypoints)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
