#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import intera_interface
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.srv import GetModelState

def obtener_posicion_cubo(nombre_modelo):
    """Le pide a Gazebo la posicion exacta del cubo en el mundo"""
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # Obtenemos el estado con respecto al marco 'world' o 'base'
        respuesta = get_model_state(nombre_modelo, 'base')
        return respuesta.pose
    except rospy.ServiceException as e:
        rospy.logerr(f"No se pudo obtener la posicion de Gazebo: {e}")
        return None

def main():
    # 1. Inicializar ROS y MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('sawyer_pick_and_place_node', anonymous=True)

    # 2. Configurar el brazo y el gripper de Sawyer
    brazo = moveit_commander.MoveGroupCommander("right_arm")
    try:
        gripper = intera_interface.Gripper('right_gripper')
        gripper.calibrate()
    except Exception as e:
        rospy.logwarn(f"No se detecto gripper fisico/simulado: {e}. Se procedera solo con el brazo.")
        gripper = None

    # Configurar tolerancias y velocidades
    brazo.set_planning_time(5.0)
    brazo.set_max_velocity_scaling_factor(0.4) # Velocidad prudente para simulación

    # ==========================================
    # CONFIGURACIÓN: Cambia estos datos a tu gusto
    # ==========================================
    NOMBRE_CUBO = "block"  # El nombre que encontraste en el Paso 1
    
    # Coordenada final a donde quieres llevar el cubo (X, Y, Z)
    X_DESTINO = 0.6
    Y_DESTINO = 0.1
    Z_DESTINO = 0.2  # Ajusta la altura para que no choque con la mesa
    # ==========================================

    # 3. Obtener la Pose del cubo dinámicamente
    rospy.loginfo("Buscando la posicion del cubo en Gazebo...")
    pose_cubo = obtener_posicion_cubo(NOMBRE_CUBO)
    if not pose_cubo:
        return

    x_cubo = pose_cubo.position.x
    y_cubo = pose_cubo.position.y
    z_cubo = pose_cubo.position.z
    rospy.loginfo(f"Cubo encontrado en: X={x_cubo:.3f}, Y={y_cubo:.3f}, Z={z_cubo:.3f}")

    # Definir una orientación estándar para el gripper (mirando hacia abajo)
    # Estos valores de cuaternión suelen funcionar bien para aproximación vertical
    orientacion_abajo = pose_cubo.orientation 
    orientacion_abajo.x = 0.0
    orientacion_abajo.y = 1.0
    orientacion_abajo.z = 0.0
    orientacion_abajo.w = 0.0

    # Altura de seguridad para no arrastrar el cubo (offset en Z)
    ALTURA_SEGURIDAD = 0.15 

    # --- FASE 1: Aproximación (Pre-Grasp) ---
    rospy.loginfo("1. Moviendo a posicion de seguridad sobre el cubo...")
    pose_objetivo = Pose()
    pose_objetivo.position.x = x_cubo
    pose_objetivo.position.y = y_cubo
    pose_objetivo.position.z = z_cubo + ALTURA_SEGURIDAD
    pose_objetivo.orientation = orientacion_abajo
    
    brazo.set_pose_target(pose_objetivo)
    if not brazo.go(wait=True):
        rospy.logerr("Error al planear aproximacion")
        return

    if gripper: gripper.open()
    rospy.sleep(1.0)

    # --- FASE 2: Bajar y Agarrar (Grasp) ---
    rospy.loginfo("2. Bajando al cubo...")
    pose_objetivo.position.z = z_cubo + 0.02 # Un pequeño margen para no golpear la mesa
    brazo.set_pose_target(pose_objetivo)
    brazo.go(wait=True)

    rospy.loginfo("Agarrando cubo...")
    if gripper: gripper.close()
    rospy.sleep(1.0)

    # --- FASE 3: Levantar (Retract) ---
    rospy.loginfo("3. Levantando cubo...")
    pose_objetivo.position.z = z_cubo + ALTURA_SEGURIDAD
    brazo.set_pose_target(pose_objetivo)
    brazo.go(wait=True)

    # --- FASE 4: Transportar a Destino ---
    rospy.loginfo("4. Transportando a la coordenada de destino...")
    pose_objetivo.position.x = X_DESTINO
    pose_objetivo.position.y = Y_DESTINO
    pose_objetivo.position.z = Z_DESTINO + ALTURA_SEGURIDAD
    brazo.set_pose_target(pose_objetivo)
    brazo.go(wait=True)

    # --- FASE 5: Depositar ---
    rospy.loginfo("5. Depositando el cubo...")
    pose_objetivo.position.z = Z_DESTINO
    brazo.set_pose_target(pose_objetivo)
    brazo.go(wait=True)

    rospy.loginfo("Soltando cubo...")
    if gripper: gripper.open()
    rospy.sleep(1.0)

    # Regresar a altura de seguridad para terminar limpio
    pose_objetivo.position.z = Z_DESTINO + ALTURA_SEGURIDAD
    brazo.set_pose_target(pose_objetivo)
    brazo.go(wait=True)

    rospy.loginfo("¡Misión cumplida, pelao de maestría!")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
