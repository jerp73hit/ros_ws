#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
import image_geometry
import math

class PoseEstimatorVision:
    def __init__(self):
        rospy.init_node('vision_pose_estimator')
        self.bridge = CvBridge()
        
        # 1. PRIMERO creamos el modelo vacío
        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_info_received = False
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_frame = "base"

        rgb_topic = "/io/internal_camera/right_hand_camera/image_raw"
        depth_topic = "/io/internal_camera/right_hand_camera/depth/image_raw"

        rospy.loginfo("Iniciando nodo de visión con Hacking de Matriz...")

        # 2. LUEGO inyectamos la matriz (ahora sí sabe dónde guardarla)
        self.force_camera_info(width=800, height=800, fov=1.047) 
        
        # 3. Y por último los suscriptores
        sub_rgb = message_filters.Subscriber(rgb_topic, Image)
        sub_depth = message_filters.Subscriber(depth_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], queue_size=50, slop=0.5)
        self.ts.registerCallback(self.vision_callback)
    def force_camera_info(self, width, height, fov):
        """Crea una matriz de cámara perfecta para Gazebo sin depender de un tópico"""
        msg = CameraInfo()
        msg.width = width
        msg.height = height
        msg.distortion_model = "plumb_bob"
        msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Calcular distancia focal basada en el Field of View (FOV)
        f = (width / 2.0) / math.tan(fov / 2.0)
        cx = width / 2.0
        cy = height / 2.0
        
        msg.K = [f, 0, cx, 
                 0, f, cy, 
                 0, 0, 1]
        msg.P = [f, 0, cx, 0, 
                 0, f, cy, 0, 
                 0, 0, 1, 0]
                 
        self.cam_model.fromCameraInfo(msg)
        self.cam_info_received = True
        rospy.loginfo("¡Matriz de cámara forzada exitosamente! (Hacking de simulación)")

    def info_callback(self, msg):
        """Guarda la matriz de la cámara y se desconecta para no gastar CPU"""
        if not self.cam_info_received:
            self.cam_model.fromCameraInfo(msg)
            self.cam_info_received = True
            rospy.loginfo("¡Intrínsecos de cámara recibidos y guardados!")
            self.info_sub.unregister() # Apagamos el suscriptor

    def mock_ai_detection(self, rgb_image):
        # Mantenemos la detección simulada de la banana
        return [{"class": "banana", "box": [610, 490, 690, 690]}]

    def estimate_yaw(self, rgb_image, box):
        xmin, ymin, xmax, ymax = box
        roi = rgb_image[ymin:ymax, xmin:xmax]
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return 0.0
            
        c = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        angle_deg = rect[2]
        
        width, height = rect[1]
        if width < height:
            angle_deg = angle_deg + 90
            
        return math.radians(angle_deg)
    def vision_callback(self, rgb_msg, depth_msg):
            if not self.cam_info_received:
                return

            camera_frame = rgb_msg.header.frame_id

            try:
                cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
                cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1") 
            except Exception as e:
                rospy.logerr(f"Error cv_bridge: {e}")
                return

            # ========================================================
            # AUTO-AJUSTE DE RESOLUCIÓN
            # Si la imagen real no es 800x800, actualizamos la matriz
            # ========================================================
            real_h, real_w = cv_rgb.shape[:2]
            if self.cam_model.width != real_w or self.cam_model.height != real_h:
                rospy.logwarn(f"Corrigiendo resolución a {real_w}x{real_h}")
                self.force_camera_info(width=real_w, height=real_h, fov=1.047)

            # La IA simulada
            detecciones = self.mock_ai_detection(cv_rgb)

            for det in detecciones:
                # Desempaquetamos los píxeles
                xmin, ymin, xmax, ymax = det["box"]
                
                # 1. Dibujamos el Bounding Box en la imagen para poder VERLO
                cv2.rectangle(cv_rgb, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(cv_rgb, det['class'], (xmin, ymin-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                u = int((xmin + xmax) / 2)
                v = int((ymin + ymax) / 2)
                
                # Dibujamos un punto rojo en el centro exacto donde estamos midiendo la profundidad
                cv2.circle(cv_rgb, (u, v), 5, (0, 0, 255), -1)

                # Extraemos la profundidad
                depth_patch = cv_depth[max(0, v-2):min(cv_depth.shape[0], v+3), 
                                    max(0, u-2):min(cv_depth.shape[1], u+3)]
                z_c = np.nanmedian(depth_patch)
                
                if np.isnan(z_c) or z_c <= 0:
                    continue

                ray = self.cam_model.projectPixelTo3dRay((u, v))
                x_c = ray[0] * (z_c / ray[2])
                y_c = ray[1] * (z_c / ray[2])

                point_cam = PointStamped()
                point_cam.header.frame_id = camera_frame
                point_cam.header.stamp = rospy.Time(0)
                point_cam.point.x = x_c
                point_cam.point.y = y_c
                point_cam.point.z = z_c

                try:
                    point_base = self.tf_buffer.transform(point_cam, self.target_frame, rospy.Duration(0.5))
                    yaw_rad = self.estimate_yaw(cv_rgb, det["box"])
                    
                    rospy.loginfo_throttle(1.0, f"[{det['class'].upper()}] X:{point_base.point.x:.3f}, Y:{point_base.point.y:.3f}, Z:{point_base.point.z:.3f} | Yaw: {math.degrees(yaw_rad):.0f}°")
                except Exception as e:
                    continue

            # ========================================================
            # MOSTRAR LA CÁMARA EN TIEMPO REAL
            # ========================================================
            cv2.imshow("Camara del Robot (Vision de IA)", cv_rgb)
            cv2.waitKey(1) # Necesario para que OpenCV actualice la ventana
if __name__ == '__main__':
    try:
        PoseEstimatorVision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
