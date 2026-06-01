#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from sensor_msgs.msg import JointState  # Required to check the topic

def main():
    # 1. Initialize moveit_commander and the ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('sawyer_sim_controller', anonymous=True)

    # 2. BULLETPROOF SYNC: Force Python to wait for Gazebo's joint states
    rospy.loginfo("Waiting for /robot/joint_states to sync with simulation time...")
    try:
        # This blocks the script until the first joint state message is received
        rospy.wait_for_message("/robot/joint_states", JointState, timeout=10.0)
        rospy.loginfo("Sync complete! Proceeding with MoveIt...")
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for joint states. Is Gazebo running?")
        sys.exit(1)

    # 3. Instantiate a RobotCommander and PlanningSceneInterface
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # 4. Connect to the specific joint group for Sawyer
    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # Give the C++ backend of move_group a tiny moment to latch its internal state
    rospy.sleep(1.0) 

    # 5. Get joint values and define the goal
    joint_goal = move_group.get_current_joint_values()
    
    # Safety check to prevent the IndexError if it still fails
    if not joint_goal:
        rospy.logerr("Joint goal is still empty! MoveIt commander failed to fetch state.")
        sys.exit(1)

    # 6. Assign the target joint values (in radians)
    joint_goal[0] = 0.0   # right_j0
    joint_goal[1] = -0.78 # right_j1
    joint_goal[2] = 0.0   # right_j2
    joint_goal[3] = 1.57  # right_j3
    joint_goal[4] = 0.0   # right_j4
    joint_goal[5] = 0.78  # right_j5
    joint_goal[6] = 0.0   # right_j6

    rospy.loginfo("Planning and executing joint trajectory...")
    
    # 7. Plan and execute the movement
    move_group.go(joint_goal, wait=True)
    
    # 8. Call stop() to guarantee no residual movement
    move_group.stop()
    rospy.loginfo("Movement execution complete!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass