#!/usr/bin/env python3

import sys
import rospy
import intera_interface
import random
from math import radians, degrees
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from trac_ik_python.trac_ik import IK
from gazebo_msgs.srv import GetModelState

# ──────────────────────────────────────────────────────────────
# CONFIGURACIÓN GENERAL
# ──────────────────────────────────────────────────────────────
VELOCITY_SCALING = 0.10     # Modo "Cirujano" (súper suave)
MOVE_TIMEOUT     = 25.0     # Más tiempo para que el PID fluya
NOMBRE_CUBO      = "block"  # Nombre de tu objeto en Gazebo

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

# Altura de seguridad para que la pinza no golpee el techo del cubo ni la mesa
Z_OFFSET_AGARRE = 0.025 

# ──────────────────────────────────────────────────────────────
# MAGIA DEL IK (trac_ik) Y MOVIMIENTO ARTICULAR DIRECTO
# ──────────────────────────────────────────────────────────────
def make_ik_solver():
    return IK(
        IK_BASE_LINK, IK_TIP_LINK,
        timeout=0.05, epsilon=0.001, 
        solve_type="Distance" # Evita que el brazo salte violentamente
    )

def solve_ik(solver, limb, x_base, y_base, z_base, roll_deg, pitch_deg, yaw_deg):
    # ¡Las coordenadas entran puras! Ya no restamos la altura de la base.
    qx, qy, qz, qw = quaternion_from_euler(
        radians(roll_deg), radians(pitch_deg), radians(yaw_deg)
    )

    current = limb.joint_angles()
    seed = [current.get(j, 0.0) for j in JOINT_NAMES]

    return solver.get_ik(
        seed, x_base, y_base, z_base, qx, qy, qz, qw,
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
# LECTOR AUTOMÁTICO DE GAZEBO EN 3D (Marco 'base')
# ──────────────────────────────────────────────────────────────
def obtener_estado_real_cubo():
    rospy.loginfo(f"Buscando a '{NOMBRE_CUBO}' en Gazebo...")
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        # OJO: Usamos 'base' para que el X, Y y Z estén alineados perfectamente al robot
        resp = get_state(NOMBRE_CUBO, 'base') 
        
        if not resp.success:
            rospy.logwarn(f"No se encontró el objeto '{NOMBRE_CUBO}'.")
            return None, None, None, None, None, None
            
        pos = resp.pose.position
        ori = resp.pose.orientation
        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        
        return pos.x, pos.y, pos.z, degrees(roll_rad), degrees(pitch_rad), degrees(yaw_rad)
    except Exception as e:
        rospy.logerr(f"Fallo al contactar Gazebo: {e}")
        return None, None, None, None, None, None

def generar_destino_valido(solver, limb, roll, pitch, yaw, z_base):
    max_intentos = 10
    for _ in range(max_intentos):
        rx = random.uniform(PLACE_X_MIN, PLACE_X_MAX)
        ry = random.uniform(PLACE_Y_MIN, PLACE_Y_MAX)
        if solve_ik(solver, limb, rx, ry, z_base, roll, pitch, yaw):
            return rx, ry
    return None, None

# ──────────────────────────────────────────────────────────────
# SECUENCIA PRINCIPAL (COREOGRAFÍA DE CIRUJANO)
# ──────────────────────────────────────────────────────────────
def main():
    rospy.init_node("sawyer_smart_pick_place", anonymous=True)

    gripper = intera_interface.Gripper("right_gripper")
    limb = intera_interface.Limb("right")
    solver = make_ik_solver()

    # Inicializar el robot en la posición segura antes de pedir la primera orden
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
        # LAS DOS POSES: AGARRE (Chueco) vs DESTINO (Recto)
        # ---------------------------------------------------------
        roll_base  = 180.0
        pitch_base = 0.0
        
        # Pose de Agarre: Copiamos el ángulo del cubo (optimizado para pinza paralela)
        yaw_agarre = cubo_yaw
        if yaw_agarre > 90.0:   
            yaw_agarre -= 180.0
        elif yaw_agarre < -90.0: 
            yaw_agarre += 180.0
            
        # Pose de Destino: Perfectamente alineado a la mesa
        yaw_destino = 180.0  

        # Dinámica de Altura Segura (15 cm por encima del cubo detectado)
        hover_z = pick_z + 0.15 
        # ---------------------------------------------------------

        # 2. Calcular destino aleatorio (Usando la pose RECTA para validar)
        place_x, place_y = generar_destino_valido(solver, limb, roll_base, pitch_base, yaw_destino, pick_z)
        if place_x is None:
            print("  ✗ El robot no alcanza ese destino con pose recta. Intenta de nuevo.")
            continue

        print(f" ✓ Origen (Relativo a Base): X={pick_x:.3f}, Y={pick_y:.3f}, Z={pick_z:.3f} | Ángulo de Agarre: {yaw_agarre:.1f}°")
        print(f" ✓ Destino Aleatorio      : X={place_x:.3f}, Y={place_y:.3f} | Ángulo Final: {yaw_destino:.1f}°")
        print("-" * 60)

        # 3. COREOGRAFÍA
        gripper.open()
        rospy.sleep(0.5)

        rospy.loginfo("1. Hover de Agarre (Alineando ángulo del cubo en el aire)...")
        if not move_to(solver, limb, pick_x, pick_y, hover_z, roll_base, pitch_base, yaw_agarre):
            rospy.logwarn("Trayectoria bloqueada. Abortando.")
            ir_a_safe_pose(limb)
            continue

        rospy.loginfo("2. Bajando recto como ascensor...")
        move_to(solver, limb, pick_x, pick_y, pick_z + Z_OFFSET_AGARRE, roll_base, pitch_base, yaw_agarre)

        rospy.loginfo("3. Cerrando pinza suavemente...")
        gripper.close()
        rospy.sleep(1.0) # TIEMPO MUERTO para física de Gazebo

        rospy.loginfo("4. Levantando recto (Manteniendo ángulo de agarre)...")
        move_to(solver, limb, pick_x, pick_y, hover_z, roll_base, pitch_base, yaw_agarre)

        rospy.loginfo("5. Viajando al destino y ENDEREZANDO la muñeca en el aire...")
        move_to(solver, limb, place_x, place_y, hover_z, roll_base, pitch_base, yaw_destino)

        rospy.loginfo("6. Depositando recto...")
        move_to(solver, limb, place_x, place_y, pick_z + Z_OFFSET_AGARRE, 0, 0, yaw_destino)

        rospy.loginfo("7. Abriendo pinza...")
        gripper.open()
        rospy.sleep(1.0) # TIEMPO MUERTO para asentar el cubo

        rospy.loginfo("8. Retirada VERTICAL (Para no barrer el cubo)...")
        move_to(solver, limb, place_x, place_y, hover_z, roll_base, pitch_base, yaw_destino)

        rospy.loginfo("9. Retirada a SAFE POSE...")
        ir_a_safe_pose(limb) 

        rospy.loginfo("✓ Operación limpia completada. Esperando próxima orden.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
