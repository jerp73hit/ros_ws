from sns_ik import move_to, make_ik_solver
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import rospy
import intera_interface
import ultralytics
from ultralytics import YOLO
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetModelState
import numpy as np
import cv2
import copy
from frame2world import get_positions

rec = None


class receiver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param(
            "~image_topic",
            "/io/internal_camera/head_camera/image_rect_color"
        )

        rospy.Subscriber(self.image_topic, Image,
                         self.image_callback, queue_size=1)

        self.frame = np.zeros((4, 4, 3), np.uint8)
        self.model = YOLO('yolo_model/weights/best.pt')

    def image_callback(self, msg):
        global rec
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.frame = copy.deepcopy(frame)
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", str(e))
            return


def init_api():
    global rec
    rec = receiver()


def make_scene() -> dict:
    results = rec.model.predict(source="./vaina.jpeg", save=False, show=False)

    get_positions(results)

    return {}


def set_gripper(state: str):
    return


def execute_waypoints(waypoints: list):

    rospy.init_node("sawyer_tracik_move", anonymous=True)

    rospy.loginfo("Opening gripper ...")
    gripper = intera_interface.Gripper("right_gripper")
    gripper.open()
    rospy.sleep(1.0)

    rospy.loginfo("Initialising limb ...")
    limb = intera_interface.Limb("right")

    rospy.loginfo("Initialising IK solver ...")
    solver = make_ik_solver()

    # while not rospy.is_shutdown():
    for waypoint in waypoints:
        rospy.sleep(0.8)
        [x, y, z] = waypoint["pos"]
        [r, p, ya] = waypoint["ori"]
        move_to(solver, limb, x, y, z, r, p, ya)
        rospy.sleep(0.8)
        set_gripper(waypoint["gripper"])


# r, p, y = euler_from_quaternion([0, 1, 0, 0])
r, p, y = 180, 0, 180
hopos = [0.565, 0.115, 0.93]
pos = [0.565, 0.115, 0.78]
waypoints = [
    {"pos": hopos, "ori": [r, p, y], "gripper": "open"},
    {"pos": pos, "ori": [r, p, y], "gripper": "close"},


]

execute_waypoints(waypoints)
