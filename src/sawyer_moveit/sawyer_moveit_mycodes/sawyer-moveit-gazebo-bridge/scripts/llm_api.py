from sns_ik import move_to, make_ik_solver
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import rospy
import intera_interface

def make_scene() -> dict:
    return

def set_gripper(state: str):
    rospy.



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
r,p,y= 180,0,180
hopos = [0.565,0.115,0.93]
pos = [0.565,0.115,0.78]
waypoints = [
    {"pos": hopos, "ori": [r,p,y], "gripper": "open"},
    {"pos": pos, "ori": [r,p,y], "gripper": "close"},
]

execute_waypoints(waypoints)
