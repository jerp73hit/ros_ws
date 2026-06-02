#!/usr/bin/env python3

import sys
import math
import rospy
import intera_interface
import random
from math import radians, degrees
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
from trac_ik_python.trac_ik import IK
from gazebo_msgs.srv import GetModelState

# ──────────────────────────────────────────────────────────────
# CONFIGURACIÓN GENERAL
# ──────────────────────────────────────────────────────────────
VELOCITY_SCALING = 0.1    # Modo "Cirujano" (súper suave)
MOVE_TIMEOUT     = 28.0     # Más tiempo para pensar y llegar
SAWYER_BASE_Z    = 0.93     
NOMBRE_CUBO      = "block"  # Nombre de tu objeto en Gazebo

# ¡VITAL! Positivo para agarrar por encima y no chocar contra la madera
Z_OFFSET = 0.012

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
SAFE_Z  = 0.93   

# ──────────────────────────────────────────────────────────────
# MAGIA DEL IK (trac_ik) Y MOVIMIENTO ARTICULAR DIRECTO
# ──────────────────────────────────────────────────────────────
def make_ik_solver():
    return IK(
        IK_BASE_LINK, IK_TIP_LINK,
        timeout=0.05, epsilon=0.001, 
        solve_type="Speed" # Evita saltos violentos
    )

def solve_ik(solver, limb, x_world, y_world, z_world, roll_deg, pitch_deg, yaw_deg):
    x_rb = x_world
    y_rb = y_world
    # Restamos la base para el cálculo con 'world'
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

def move_to(solver, limb, x, y, z, roll_deg, pitch_deg, yaw_deg, velocidad=None):
    # Si no especificamos velocidad, usa la configuración general (0.08)
    if velocidad is None:
        velocidad = VELOCITY_SCALING

    solution = solve_ik(solver, limb, x, y, z, roll_deg, pitch_deg, yaw_deg)
    if solution is None:
        return False
    
    joint_goal = dict(zip(JOINT_NAMES, solution))
    # Aplicamos la velocidad específica para este movimiento
    limb.set_joint_position_speed(velocidad)
    limb.move_to_joint_positions(joint_goal, timeout=MOVE_TIMEOUT)
    return True

def ir_a_safe_pose(limb):
    """Mueve los 7 motores directamente a la posición de descanso definida"""
    rospy.loginfo("Moviendo a SAFE POSE articular...")
    joint_goal = dict(zip(JOINT_NAMES, SAFE_POSE))
    limb.set_joint_position_speed(VELOCITY_SCALING)
    limb.move_to_joint_positions(joint_goal, timeout=MOVE_TIMEOUT)

# ──────────────────────────────────────────────────────────────
# LECTOR 3D DE GAZEBO (MATEMÁTICA DE VECTORES DIRECTORES)
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
        
        # Inmunidad al Roll/Pitch: Extraemos la matriz de rotación
        q = [ori.x, ori.y, ori.z, ori.w]
        R = quaternion_matrix(q)
        
        # Vectores directores de los ejes locales del cubo
        vec_x = [R[0,0], R[1,0], R[2,0]]
        vec_y = [R[0,1], R[1,1], R[2,1]]
        vec_z = [R[0,2], R[1,2], R[2,2]]
        
        # Encontramos el eje que esté más "acostado" sobre la mesa (Z más pequeño)
        ejes = [vec_x, vec_y, vec_z]
        eje_plano = min(ejes, key=lambda v: abs(v[2]))
        
        # Calculamos el verdadero ángulo horizontal de ese eje
        true_yaw_rad = math.atan2(eje_plano[1], eje_plano[0])
        true_yaw_deg = math.degrees(true_yaw_rad)
        
        # Retornamos pos y un Yaw limpio, ignorando Roll y Pitch para bajar en modo grúa
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

def mover_en_linea_recta_z(solver, limb, x, y, z_inicio, z_fin, roll, pitch, yaw, pasos=10):
    """Fuerza al brazo a bajar/subir por un tubo vertical imaginario sin contorsiones"""
    distancia = z_fin - z_inicio
    incremento = distancia / pasos
    
    for i in range(1, pasos + 1):
        z_actual = z_inicio + (incremento * i)
        # Movemos cada pedacito a velocidad controlada
        exito = move_to(solver, limb, x, y, z_actual, roll, pitch, yaw, velocidad=0.05)
        if not exito:
            return False
        # Pausa microscópica para suavizar el flujo de los motores
        rospy.sleep(0.05) 
    return True

# ──────────────────────────────────────────────────────────────
# SECUENCIA PRINCIPAL (CON CÁLCULO DE DELTA)
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
        pick_x, pick_y, pick_z, _, _, cubo_yaw = obtener_estado_real_cubo()
        if pick_x is None: continue
            
        roll_base  = 180.0
        pitch_base = 0.0
        yaw_destino = 180.0  

        place_x, place_y = generar_destino_valido(solver, limb, roll_base, pitch_base, yaw_destino, pick_z)
        if place_x is None:
            print("  ✗ El robot no alcanza ese destino con pose recta. Intenta de nuevo.")
            continue

        print(f" ✓ Origen: X={pick_x:.3f}, Y={pick_y:.3f} | Ángulo Analizado: {cubo_yaw:.1f}°")
        print("-" * 60)

        # =========================================================
        # FASE 1: APROXIMACIÓN
        # =========================================================
        gripper.open()
        rospy.sleep(0.5)

        rospy.loginfo("1. Viajando sobre el cubo (Hover en X, Y)...")
        if not move_to(solver, limb, pick_x, pick_y, SAFE_Z, roll_base, pitch_base, 0.0):
            rospy.logwarn("Trayectoria bloqueada. Abortando.")
            ir_a_safe_pose(limb)
            continue
        
        # ¡PARADA TÁCTICA! Frena en seco y disipa la inercia del viaje largo
        rospy.sleep(0.8) 

        # =========================================================
        # FASE 2: ALINEACIÓN FINA
        # =========================================================
        rospy.loginfo("2. Calculando Delta de alineación...")
        pose_actual = limb.endpoint_pose()
        q_actual = pose_actual['orientation']
        _, _, yaw_pinza_rad = euler_from_quaternion([q_actual.x, q_actual.y, q_actual.z, q_actual.w])
        yaw_pinza = degrees(yaw_pinza_rad)
        
        delta_yaw = cubo_yaw - yaw_pinza

        while delta_yaw > 45.0:   delta_yaw -= 90.0
        while delta_yaw < -45.0:  delta_yaw += 90.0

        nuevo_yaw_pinza = yaw_pinza + delta_yaw
        rospy.loginfo(f"   -> Ajustando muñeca {delta_yaw:.1f}° en el aire...")
        
        move_to(solver, limb, pick_x, pick_y, SAFE_Z, roll_base, pitch_base, nuevo_yaw_pinza)
        
        # ¡PARADA TÁCTICA! Esperar que la muñeca termine de rotar antes de bajar
        rospy.sleep(0.5)

# =========================================================
        # FASE 3: EXTRACCIÓN (Descenso en Z Lineal)
        # =========================================================
        rospy.loginfo("3. Bajando a Z de agarre (LÍNEA RECTA PERFECTA)...")
        # Aquí reemplazamos move_to por mover_en_linea_recta_z
        mover_en_linea_recta_z(
            solver, limb, pick_x, pick_y, 
            SAFE_Z, pick_z + Z_OFFSET,  # Va desde arriba hacia abajo
            roll_base, pitch_base, nuevo_yaw_pinza, 
            pasos=10 # Divide la bajada en 10 mini-pasos
        )
        
         

        rospy.loginfo("4. Cerrando pinza...")
        gripper.close()
         

        rospy.loginfo("5. Levantando verticalmente (LÍNEA RECTA PERFECTA)...")
        # Aquí también subimos en línea recta para no jalar el cubo de lado
        mover_en_linea_recta_z(
            solver, limb, pick_x, pick_y, 
            pick_z + Z_OFFSET, SAFE_Z,  # Va desde abajo hacia arriba
            roll_base, pitch_base, nuevo_yaw_pinza, 
            pasos=10
        )
        
        
        # =========================================================
        # FASE 4: DEPÓSITO
        # =========================================================
        rospy.loginfo("6. Viajando al destino y orientando a pose final...")
        move_to(solver, limb, place_x, place_y, SAFE_Z, roll_base, pitch_base, yaw_destino)
        
        

        rospy.loginfo("7. Depositando recto...")
        # Evitamos que estrelle el cubo contra la mesa al soltarlo
        mover_en_linea_recta_z(solver, limb, place_x, place_y, SAFE_Z, pick_z + Z_OFFSET, roll_base, pitch_base, yaw_destino, pasos=10)
        
        
        
        rospy.loginfo("8. Abriendo pinza...")
        gripper.open()
        

        rospy.loginfo("9. Retirada a SAFE POSE...")
        mover_en_linea_recta_z(solver, limb, place_x, place_y, pick_z+Z_OFFSET, SAFE_Z, roll_base, pitch_base, yaw_destino, pasos=10)
        
        ir_a_safe_pose(limb) 

        rospy.loginfo("✓ Operación lista.")
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
