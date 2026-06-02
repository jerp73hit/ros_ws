#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import intera_interface
from math import sqrt
from intera_core_msgs.msg import EndpointState

# ──────────────────────────────────────────────────────────────
# CONFIGURACIÓN DE VELOCIDAD
# ──────────────────────────────────────────────────────────────
VELOCITY_SCALING     = 0.20  # Un poco más rápido para simulación
ACCELERATION_SCALING = 0.20

DURATION_SCALING = 10.0    
DURATION_MARGIN  = 30.0   

SAWYER_BASE_Z = 0.93   

# ──────────────────────────────────────────────────────────────
# LÍMITES DEL WORKSPACE
# ──────────────────────────────────────────────────────────────
X_MIN, X_MAX =  0.30,  0.90
Y_MIN, Y_MAX = -0.60,  0.60
Z_MIN, Z_MAX = -0.20,  0.80   

MIN_MOVE_DIST = 0.025   
MAX_MOVE_DIST = 12.0    

# ──────────────────────────────────────────────────────────────
# CONSTANTES
# ──────────────────────────────────────────────────────────────
TABLE_NAME = "cafe_table"   
BLOCK_NAME = "block"        
EEF_LINK   = "right_gripper_tip"
MOVE_GROUP = "right_arm"
ENDPOINT_TOPIC = "/robot/limb/right/endpoint_state"

# ──────────────────────────────────────────────────────────────
# LECTURA DE POSICIÓN
# ──────────────────────────────────────────────────────────────
def get_current_eef_world_pos():
    try:
        msg = rospy.wait_for_message(ENDPOINT_TOPIC, EndpointState, timeout=5.0)
        p = msg.pose.position
        world_z = p.z + SAWYER_BASE_Z
        return p.x, p.y, world_z
    except rospy.ROSException as e:
        rospy.logwarn(f"No se pudo leer {ENDPOINT_TOPIC}: {e}")
        return None, None, None

def validate_coordinates(tx, ty, tz):
    if not (X_MIN <= tx <= X_MAX): return False, f"X={tx:.3f} fuera de rango"
    if not (Y_MIN <= ty <= Y_MAX): return False, f"Y={ty:.3f} fuera de rango"
    if not (Z_MIN <= tz <= Z_MAX): return False, f"Z={tz:.3f} fuera de rango"

    cx, cy, cz = get_current_eef_world_pos()
    if cx is None: return True, "OK (salto de chequeo de distancia)"

    dist = sqrt((tx - cx)**2 + (ty - cy)**2 + (tz - cz)**2)
    if dist < MIN_MOVE_DIST: return False, f"Ya estás en el objetivo (dist={dist:.4f}m)"
    if dist > MAX_MOVE_DIST: return False, "Objetivo ridículamente lejos"

    return True, f"OK (dist={dist:.3f} m)"

# ──────────────────────────────────────────────────────────────
# INTERFAZ DE USUARIO
# ──────────────────────────────────────────────────────────────
def ask_coordinates():
    while True:
        print("\n" + "─" * 55)
        print(" Ingresa objetivo (coordenadas de mundo) — 'q' para salir")
        coords = {}
        quit_req = False

        for axis in ("x", "y", "z"):
            while True:
                raw = input(f"  {axis.upper()}: ").strip().lower()
                if raw == "q":
                    quit_req = True
                    break
                try:
                    coords[axis] = float(raw)
                    break
                except ValueError:
                    print("  ⚠ Ingresa un número válido")
            if quit_req: break

        if quit_req:
            print("Apagando...")
            sys.exit(0)

        tx, ty, tz = coords["x"], coords["y"], coords["z"]
        ok, reason = validate_coordinates(tx, ty, tz)

        if ok:
            print(f"  ✓ Aceptado ({tx:.3f}, {ty:.3f}, {tz:.3f}) — {reason}")
            return tx, ty, tz
        else:
            print(f"  ✗ Rechazado: {reason}\n  Intenta de nuevo.")

# ──────────────────────────────────────────────────────────────
# MAGIA DE MOVIMIENTO Y CINEMÁTICA
# ──────────────────────────────────────────────────────────────
def build_pose(arm_group, x, y, z):
    pose = arm_group.get_current_pose(EEF_LINK).pose
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    
    # EL TRUCO DE ORO: Forzar orientación hacia la mesa (Garra hacia abajo)
    # Esto evita que el brazo se tranque intentando mantener ángulos imposibles
    pose.orientation.x = 0.0
    pose.orientation.y = 1.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
    
    return pose

def move_to(arm_group, scene, x, y, z):
    arm_group.set_start_state_to_current_state()
    rospy.loginfo(f"Moviendo a ({x:.3f}, {y:.3f}, {z:.3f}) con SNS-IK...")

    # Tolerancias un poco más amigables para el resolvedor
    arm_group.set_goal_orientation_tolerance(0.05) 
    arm_group.set_goal_position_tolerance(0.005)     

    arm_group.set_pose_target(build_pose(arm_group, x, y, z), EEF_LINK)
    success = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    arm_group.set_goal_orientation_tolerance(0.01)
    arm_group.set_goal_position_tolerance(0.001)

    if success:
        rospy.loginfo("✓ Objetivo alcanzado.")
    else:
        rospy.logwarn("✗ El planificador SNS-IK no pudo resolver esta ruta.")
    return success

# ──────────────────────────────────────────────────────────────
# CONFIGURACIÓN INICIAL
# ──────────────────────────────────────────────────────────────
def setup_moveit():
    arm_group = moveit_commander.MoveGroupCommander(MOVE_GROUP)
    arm_group.set_end_effector_link(EEF_LINK)
    arm_group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
    arm_group.set_max_acceleration_scaling_factor(ACCELERATION_SCALING)
    arm_group.set_planning_time(5.0) # SNS-IK es rápido, no necesita 20s
    arm_group.set_num_planning_attempts(10)

    rospy.set_param("/move_group/trajectory_execution/allowed_execution_duration_scaling", DURATION_SCALING)
    rospy.set_param("/move_group/trajectory_execution/allowed_goal_duration_margin", DURATION_MARGIN)
    return arm_group

def setup_scene(arm_group, scene):
    rospy.loginfo("Limpiando mesa y bloque del mapa de colisiones...")
    scene.remove_world_object(TABLE_NAME)  
    scene.remove_world_object(BLOCK_NAME)
    rospy.sleep(1.0)
    arm_group.set_start_state_to_current_state()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sawyer_sns_interactive", anonymous=True)

    rospy.loginfo("Abriendo gripper...")
    gripper = intera_interface.Gripper("right_gripper")
    gripper.open()
    rospy.sleep(1.0)

    arm_group = setup_moveit()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)
    setup_scene(arm_group, scene)

    print("\n" + "=" * 55)
    print(" Sawyer — Posicionamiento Interactivo (SNS-IK Powered)")
    print("=" * 55)

    while not rospy.is_shutdown():
        x, y, z = ask_coordinates()
        move_to(arm_group, scene, x, y, z)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
