#!/usr/bin/env python3

import sys
import rospy
import math
import intera_interface
import random
from math import radians, degrees
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
from trac_ik_python.trac_ik import IK
from gazebo_msgs.srv import GetModelState

# ──────────────────────────────────────────────────────────────
# CONFIGURACIÓN GENERAL
# ──────────────────────────────────────────────────────────────
VELOCITY_SCALING = 0.08     # Modo "Cirujano" (súper suave)
MOVE_TIMEOUT     = 30.0     # Más tiempo para pensar y llegar
SAWYER_BASE_Z    = 0.93     
NOMBRE_CUBO      = "block"  # Nombre de tu objeto en Gazebo

# ¡VITAL! Positivo para agarrar por encima y no chocar contra la madera
Z_OFFSET = 0.015 

IK_BASE_LINK = "base"
IK_TIP_LINK  = "right_gripper_tip"

JOINT_NAMES = [
    "right_j0", "right_j1", "right_j2", "right_j3",
    "right_j4", "right_j5", "right_j6"
]

# Posición segura de descanso (Standby)
SAFE_POSE = [
    -0.041662954890248294,
    -1.0258291091425074,
     0.0293680414401436,
     2.17518162913313,
    -0.06703022873354225,
     0.3968371433926965,
     1.7659649178699421,
]

PLACE_X_MIN, PLACE_X_MAX =  0.45, 0.75
PLACE_Y_MIN, PLACE_Y_MAX = -0.30, 0.30
SAFE_Z  = 0.95   

# ──────────────────────────────────────────────────────────────
# MAGIA DEL IK (trac_ik) Y MOVIMIENTO ARTICULAR DIRECTO
# ──────────────────────────────────────────────────────────────
def make_ik_solver():
    return IK(
        IK_BASE_LINK, IK_TIP_LINK,
        timeout=0.05, epsilon=0.001, 
        solve_type="Distance" # Evita saltos violentos
    )

def solve_ik(solver, limb, x_world, y_world, z_world, roll_deg, pitch_deg, yaw_deg):
    x_rb = x_world
    y_rb = y_world
    # Restamos la base para volver al cálculo original que sí funcionaba
    z_rb = z_world - SAWYER_BASE_Z 

    qx, qy, qz, qw = quaternion_from_euler(
        radians(roll_deg), radians(pitch_deg), radians(yaw_deg)
    )

    current = limb.joint_angles()
    seed = [current.get(j, 0.0) for j in JOINT_NAMES]

    return solver.get_ik(
        seed, x_rb, y_rb, z_rb, qx, qy, qz, qw,
        bx=0.001, by=0.001, bz=0.001,
        brx=0.1,  bry=0.1,  brz=0.1
    )

def move_to(solver, limb, x, y, z, roll_deg, pitch_deg, yaw_deg):
    solution = solve_ik(solver, limb, x, y, z, roll_deg, pitch_deg, yaw_deg)
    if solution is None:
        return False
    
    joint_goal = dict(zip(JOINT_NAMES, solution))
    limb.set_joint_position_speed(VELOCITY_SCALING)
    limb.move_to_joint_positions(joint_goal, timeout=MOVE_TIMEOUT)
    return True

def ir_a_safe_pose(limb):
    """Mueve los 7 motores directamente a la posición de descanso definida"""
    rospy.loginfo("Moviendo a SAFE POSE articular...")
    joint_goal = dict(zip(JOINT_NAMES, SAFE_POSE))
    limb.set_joint_position_speed(VELOCITY_SCALING)
    limb.move_to_joint_positions(joint_goal, timeout=MOVE_TIMEOUT)

# ──────────────────────────────────────────────────────────────
# LECTOR AUTOMÁTICO DE GAZEBO EN 3D
# ──────────────────────────────────────────────────────────────
def obtener_estado_real_cubo():
    rospy.loginfo(f"Buscando a '{NOMBRE_CUBO}' en Gazebo...")
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_state(NOMBRE_CUBO, 'world') 
        
        if not resp.success:
            rospy.logwarn(f"No se encontró el objeto '{NOMBRE_CUBO}'.")
            return None, None, None, None, None, None
            
        pos = resp.pose.position
        ori = resp.pose.orientation
        
        # 1. Convertir cuaternión a Matriz de Rotación 4x4
        q = [ori.x, ori.y, ori.z, ori.w]
        R = quaternion_matrix(q)
        
        # 2. Extraer los vectores directores de los ejes locales del cubo
        vec_x = [R[0,0], R[1,0], R[2,0]]
        vec_y = [R[0,1], R[1,1], R[2,1]]
        vec_z = [R[0,2], R[1,2], R[2,2]]
        
        # 3. Encontrar cuál eje está acostado en la mesa (el que tenga menor Z)
        ejes = [vec_x, vec_y, vec_z]
        eje_plano = min(ejes, key=lambda v: abs(v[2]))
        
        # 4. Calcular el verdadero ángulo horizontal de la cara plana
        true_yaw_rad = math.atan2(eje_plano[1], eje_plano[0])
        true_yaw_deg = math.degrees(true_yaw_rad)
        
        # Retornamos el Yaw perfecto (ignoramos roll y pitch porque bajaremos en modo grúa)
        return pos.x, pos.y, pos.z, 0.0, 0.0, true_yaw_deg
        
    except Exception as e:
        rospy.logerr(f"Fallo al contactar Gazebo: {e}")
        return None, None, None, None, None, None
def generar_destino_valido(solver, limb, roll, pitch, yaw, z_mundo):
    max_intentos = 10
    for _ in range(max_intentos):
        rx = random.uniform(PLACE_X_MIN, PLACE_X_MAX)
        ry = random.uniform(PLACE_Y_MIN, PLACE_Y_MAX)
        if solve_ik(solver, limb, rx, ry, z_mundo, roll, pitch, yaw):
            return rx, ry
    return None, None

# ──────────────────────────────────────────────────────────────
# SECUENCIA PRINCIPAL
# ──────────────────────────────────────────────────────────────
def main():
    rospy.init_node("sawyer_smart_pick_place", anonymous=True)

    gripper = intera_interface.Gripper("right_gripper")
    limb = intera_interface.Limb("right")
    solver = make_ik_solver()

    rospy.loginfo("Inicializando brazo en SAFE POSE...")
    gripper.open()
    ir_a_safe_pose(limb)

    while not rospy.is_shutdown():
        print("\n" + "=" * 60)
        raw = input(" Presiona ENTER para leer el cubo y ejecutar (o 'q' para salir): ")
        if raw.strip().lower() == 'q':
            sys.exit(0)

        # 1. Leer realidad en 3D
        pick_x, pick_y, pick_z, cubo_roll, cubo_pitch, cubo_yaw = obtener_estado_real_cubo()
        if pick_x is None: continue
            
        # ---------------------------------------------------------
        # AQUÍ DEFINIMOS LAS BASES (Para evitar el NameError)
        # ---------------------------------------------------------
        roll_base  = 180.0
        pitch_base = 0.0
        yaw_destino = 180.0  
        
        # Pose de Agarre: Encontrar la cara plana más cercana (Múltiplos de 90°)
        yaw_agarre = cubo_yaw
        while yaw_agarre > 45.0:   
            yaw_agarre -= 90.0
        while yaw_agarre < -45.0: 
            yaw_agarre += 90.0
        # ---------------------------------------------------------

        # 2. Calcular destino aleatorio
        place_x, place_y = generar_destino_valido(solver, limb, roll_base, pitch_base, yaw_destino, pick_z)
        if place_x is None:
            print("  ✗ El robot no alcanza ese destino con pose recta. Intenta de nuevo.")
            continue

        print(f" ✓ Origen: X={pick_x:.3f}, Y={pick_y:.3f} | Ángulo de Agarre: {yaw_agarre:.1f}°")
        print(f" ✓ Destino: X={place_x:.3f}, Y={place_y:.3f} | Ángulo Final: {yaw_destino:.1f}°")
        print("-" * 60)

        # 3. COREOGRAFÍA DESACOPLADA (Viajar primero, girar después)
        gripper.open()
        rospy.sleep(0.5)

        rospy.loginfo("1a. Alineando muñeca en SAFE POSE (Para no chocar)...")
        if not move_to(solver, limb, pick_x, pick_y, SAFE_Z, roll_base, pitch_base, yaw_agarre):
            rospy.logwarn("Trayectoria bloqueada al alinear. Abortando.")
            ir_a_safe_pose(limb)
            continue

        rospy.loginfo("2. Bajando recto como ascensor al cubo...")
        move_to(solver, limb, pick_x, pick_y, pick_z + Z_OFFSET, roll_base, pitch_base, yaw_agarre)

        rospy.loginfo("3. Cerrando pinza suavemente...")
        gripper.close()
        rospy.sleep(1.0) 

        rospy.loginfo("4. Levantando recto (Manteniendo ángulo chueco)...")
        move_to(solver, limb, pick_x, pick_y, SAFE_Z, roll_base, pitch_base, yaw_agarre)

        rospy.loginfo("5a. Viajando al destino (Estable)...")
        move_to(solver, limb, place_x, place_y, SAFE_Z, roll_base, pitch_base, yaw_agarre)

        rospy.loginfo("5b. Enderezando la muñeca sobre el objetivo (Quieto)...")
        move_to(solver, limb, place_x, place_y, SAFE_Z, roll_base, pitch_base, yaw_destino)

        rospy.loginfo("6. Depositando recto...")
        move_to(solver, limb, place_x, place_y, pick_z + Z_OFFSET, roll_base, pitch_base, yaw_destino)

        rospy.loginfo("7. Abriendo pinza...")
        gripper.open()
        rospy.sleep(1.0) 

        rospy.loginfo("8. Retirada VERTICAL (Para no barrer el cubo)...")
        move_to(solver, limb, place_x, place_y, SAFE_Z, roll_base, pitch_base, yaw_destino)

        rospy.loginfo("9. Retirada a SAFE POSE...")
        ir_a_safe_pose(limb) 

        rospy.loginfo("✓ Operación limpia completada. Esperando próxima orden.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
