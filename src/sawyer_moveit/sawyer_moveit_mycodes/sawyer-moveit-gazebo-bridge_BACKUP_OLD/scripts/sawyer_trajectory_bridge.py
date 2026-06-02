#!/usr/bin/env python3

import rospy
import actionlib
import intera_interface

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from intera_core_msgs.msg import JointCommand


class SawyerTrajectoryBridge:
    def __init__(self):
        rospy.loginfo("Enabling Sawyer robot...")
        self.rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self.rs.enable()
        rospy.sleep(1.0)

        self.pub = rospy.Publisher(
            "/robot/limb/right/joint_command",
            JointCommand,
            queue_size=10
        )

        self.server = actionlib.SimpleActionServer(
            "/robot/limb/right/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )

        self.server.start()

        rospy.loginfo("Sawyer trajectory bridge ready.")
        rospy.loginfo("Action server: /robot/limb/right/follow_joint_trajectory")
        rospy.loginfo("Publishing to: /robot/limb/right/joint_command")

    def publish_position(self, joint_names, positions):
        cmd = JointCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.mode = JointCommand.POSITION_MODE
        cmd.names = list(joint_names)
        cmd.position = list(positions)
        self.pub.publish(cmd)

    def execute_cb(self, goal):
        result = FollowJointTrajectoryResult()
        traj = goal.trajectory

        rospy.loginfo("Received trajectory with %d points.", len(traj.points))
        rospy.loginfo("Joint names: %s", traj.joint_names)

        if not traj.joint_names:
            rospy.logerr("Trajectory has no joint names.")
            result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            self.server.set_aborted(result)
            return

        if not traj.points:
            rospy.logerr("Trajectory has no points.")
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self.server.set_aborted(result)
            return

        rate = rospy.Rate(50)
        start_time = rospy.Time.now()

        previous_positions = list(traj.points[0].positions)
        previous_time = rospy.Duration(0.0)

        for i, point in enumerate(traj.points):
            if rospy.is_shutdown():
                return

            if self.server.is_preempt_requested():
                rospy.logwarn("Trajectory preempted.")
                self.server.set_preempted()
                return

            target_positions = list(point.positions)
            target_time = point.time_from_start

            segment_duration = (target_time - previous_time).to_sec()

            if segment_duration <= 0:
                self.publish_position(traj.joint_names, target_positions)
                previous_positions = target_positions
                previous_time = target_time
                continue

            segment_start = rospy.Time.now()

            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - segment_start).to_sec()
                alpha = min(elapsed / segment_duration, 1.0)

                interpolated = []
                for p0, p1 in zip(previous_positions, target_positions):
                    interpolated.append(p0 + alpha * (p1 - p0))

                self.publish_position(traj.joint_names, interpolated)

                if alpha >= 1.0:
                    break

                rate.sleep()

            previous_positions = target_positions
            previous_time = target_time

        # Mantener el último punto un momento para que el controlador lo alcance
        final_positions = list(traj.points[-1].positions)
        hold_start = rospy.Time.now()
        while (rospy.Time.now() - hold_start).to_sec() < 1.0:
            self.publish_position(traj.joint_names, final_positions)
            rate.sleep()

        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.server.set_succeeded(result)
        rospy.loginfo("Trajectory execution finished.")


if __name__ == "__main__":
    rospy.init_node("sawyer_trajectory_bridge")
    SawyerTrajectoryBridge()
    rospy.spin()
