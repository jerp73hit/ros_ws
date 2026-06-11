#!/usr/bin/env python3
import math
import rospy
import numpy as np
from tf.transformations import quaternion_matrix
from gazebo_msgs.srv import GetModelState
from llm_api import init_api, make_scene
from frame2world import OBJECT_CLASSES

init_api()
rospy.sleep(1.0)

scene = make_scene()
objs = scene["objects"]
if not objs:
    print("[test] No objects detected.")
    exit(0)

rospy.wait_for_service("/gazebo/get_model_state")
get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

print(f"{'Object':22s} {'Vision yaw':>10s} {'Gazebo yaw':>10s} {'Error':>8s}")
print("-" * 52)

total_err = 0.0
count = 0

for name in sorted(OBJECT_CLASSES):
    try:
        resp = get_state(name, "world")
    except Exception:
        continue
    if not resp.success:
        continue

    q = resp.pose.orientation
    p = resp.pose.position

    # Gazebo yaw: direction of the most horizontal axis
    R = quaternion_matrix([q.x, q.y, q.z, q.w])
    vecs = [R[:3, 0], R[:3, 1], R[:3, 2]]
    flat = min(vecs, key=lambda v: abs(v[2]))
    gazebo_yaw = math.degrees(math.atan2(flat[1], flat[0]))

    if name not in objs:
        print(f"{name:22s} {'--':>10s} {gazebo_yaw:>+8.1f}°  {'--':>8s}  (not in vision)")
        continue

    vision_yaw = objs[name]["orientation"]
    err = abs(vision_yaw - gazebo_yaw)
    # Normalise to [-180, 180]
    err = min(err, 360.0 - err)
    total_err += err
    count += 1

    print(f"{name:22s} {vision_yaw:>+8.1f}° {gazebo_yaw:>+8.1f}° {err:>6.1f}°")

if count > 0:
    print("-" * 52)
    print(f"{'Mean error':22s} {'':>10s} {'':>10s} {total_err/count:>6.1f}°")
