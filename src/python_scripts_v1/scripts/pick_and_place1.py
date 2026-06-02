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

VELOCITY_SCALING = 0.25     

MOVE_TIMEOUT     = 20.0     

SAWYER_BASE_Z    = 0.93     

NOMBRE_CUBO      = "block"  # El nombre de tu cubo en Gazebo


IK_BASE_LINK = "base"

IK_TIP_LINK  = "right_gripper_tip"

JOINT_NAMES  = [

    "right_j0", "right_j1", "right_j2", "right_j3",

    "right_j4", "right_j5", "right_j6"

]


PLACE_X_MIN, PLACE_X_MAX =  0.45, 0.75

PLACE_Y_MIN, PLACE_Y_MAX = -0.30, 0.30

PLACE_Z = 0.794  

SAFE_Z  = 0.95   


# ──────────────────────────────────────────────────────────────

# MAGIA DEL IK (trac_ik) - AHORA EN MODO "DISTANCE"

# ──────────────────────────────────────────────────────────────

def make_ik_solver():

    return IK(

        IK_BASE_LINK, IK_TIP_LINK,

        timeout=0.05, epsilon=0.001, 

        solve_type="Distance" # CLAVE: Evita que el brazo haga movimientos bruscos

    )


def solve_ik(solver, limb, x_world, y_world, z_world, roll_deg, pitch_deg, yaw_deg):

    x_rb = x_world

    y_rb = y_world

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


# ──────────────────────────────────────────────────────────────

# LECTOR AUTOMÁTICO DE GAZEBO

# ──────────────────────────────────────────────────────────────

def obtener_estado_real_cubo():

    """Le pregunta a Gazebo la posición X,Y,Z y el ángulo de caída del cubo"""

    rospy.loginfo(f"Buscando a '{NOMBRE_CUBO}' en Gazebo...")

    rospy.wait_for_service('/gazebo/get_model_state')

    

    try:

        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        resp = get_state(NOMBRE_CUBO, 'base')

        

        if not resp.success:

            rospy.logwarn(f"No se encontró el objeto '{NOMBRE_CUBO}'.")

            return None, None, None, None

            

        pos = resp.pose.position

        ori = resp.pose.orientation

        

        # Extraer el giro (Yaw) en el que quedó el cubo al caer

        _, _, yaw_rad = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        yaw_deg = degrees(yaw_rad)

        

        return pos.x, pos.y, pos.z, yaw_deg

        

    except Exception as e:

        rospy.logerr(f"Fallo al contactar Gazebo: {e}")

        return None, None, None, None


def generar_destino_valido(solver, limb, roll, pitch, yaw):

    max_intentos = 10

    for _ in range(max_intentos):

        rx = random.uniform(PLACE_X_MIN, PLACE_X_MAX)

        ry = random.uniform(PLACE_Y_MIN, PLACE_Y_MAX)

        

        if solve_ik(solver, limb, rx, ry, PLACE_Z, roll, pitch, yaw):

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


    while not rospy.is_shutdown():

        print("\n" + "=" * 60)

        raw = input(" Presiona ENTER para leer el cubo y ejecutar (o 'q' para salir): ")

        if raw.strip().lower() == 'q':

            sys.exit(0)


        # 1. Leer realidad

        pick_x, pick_y, pick_z, yaw_cubo = obtener_estado_real_cubo()

        

        if pick_x is None:

            continue

            

        # Forzar la garra hacia abajo (Roll 180, Pitch 0), pero copiando el giro (Yaw) del cubo

        roll_agarre = 180.0

        pitch_agarre = 0.0

        yaw_agarre = yaw_cubo # ¡El secreto para que la pinza encaje perfecto!


        # 2. Calcular destino aleatorio usando el mismo ángulo

        place_x, place_y = generar_destino_valido(solver, limb, roll_agarre, pitch_agarre, yaw_agarre)

        

        if place_x is None:

            print("  ✗ El robot está muy enredado para alcanzar un destino. Intenta mover el brazo un poco.")

            continue


        print(f" ✓ Origen detectado: X={pick_x:.3f}, Y={pick_y:.3f}, Z={pick_z:.3f} | Ángulo: {yaw_agarre:.1f}°")

        print(f" ✓ Destino asignado: X={place_x:.3f}, Y={place_y:.3f}")

        print(" Ejecutando Pick & Place Inteligente...")

        print("-" * 60)


        # 3. Empieza la coreografía

        gripper.open()

        rospy.sleep(0.5)


        rospy.loginfo("1. Aproximación alineada con el cubo...")

        if not move_to(solver, limb, pick_x, pick_y, SAFE_Z, roll_agarre, pitch_agarre, yaw_agarre):

            rospy.logwarn("Trayectoria bloqueada. Abortando.")

            continue


        rospy.loginfo("2. Bajando al cubo...")

        move_to(solver, limb, pick_x, pick_y, pick_z + 0.01, roll_agarre, pitch_agarre, yaw_agarre) # +0.01 evita rozar la mesa


        rospy.loginfo("3. Agarrando...")

        gripper.close()

        rospy.sleep(1.0)


        rospy.loginfo("4. Levantando...")

        move_to(solver, limb, pick_x, pick_y, SAFE_Z, roll_agarre, pitch_agarre, yaw_agarre)


        rospy.loginfo("5. Viajando al destino...")

        move_to(solver, limb, place_x, place_y, SAFE_Z, roll_agarre, pitch_agarre, yaw_agarre)


        rospy.loginfo("6. Depositando suavemente...")

        move_to(solver, limb, place_x, place_y, PLACE_Z, roll_agarre, pitch_agarre, yaw_agarre)


        rospy.loginfo("7. Soltando...")

        gripper.open()

        rospy.sleep(1.0)


        rospy.loginfo("8. Retirada a posición segura...")

        move_to(solver, limb, place_x, place_y, SAFE_Z, roll_agarre, pitch_agarre, yaw_agarre)


        rospy.loginfo("✓ Ciclo completado. El cubo tiene nueva pose.")


if __name__ == "__main__":

    try:

        main()

    except rospy.ROSInterruptException:

        pass
