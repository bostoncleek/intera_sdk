#!/usr/bin/env python3

# Boston Cleek
# 9/21/20
# Action server for Sawyer gripper

import rospy
import actionlib
import intera_interface
from math import fabs
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandFeedback,
    GripperCommandResult,
)


class GripperActionServer(object):
    def __init__(self):
        rospy.loginfo("Gripper effort command is ignored in Sawyer SDK")
        self._ns= "/robot/end_effector/right/gripper_action"
        self._gripper = intera_interface.Gripper()
        self._server = actionlib.SimpleActionServer(
            self._ns,
            GripperCommandAction,
            execute_cb=self._on_gripper_action,
            auto_start=False)
        self._action_name = rospy.get_name()
        self._enable = intera_interface.RobotEnable()
        self._server.start()

        if not self._gripper.is_calibrated():
            rospy.loginfo("Calibrating gripper")
            self._gripper.calibrate()

        # Action Feedback/Result
        self._fdbk = GripperCommandFeedback()
        self._result = GripperCommandResult()

        self._max_pos = self._gripper.MAX_POSITION
        self._min_pos = self._gripper.MIN_POSITION
        # self._max_vel = self._gripper.MAX_VELOCITY
        # self._min_vel = self._gripper.MIN_VELOCITY
        # self._gripper.set_dead_zone(0.003)
        self._dead_zone = self._gripper.get_dead_zone()
        print("Dead zone: ", self._dead_zone)
        self._timeout = 5.0


    def _check_state(self, position):
        # Closing but grasp aperature is greater than expected
        if (self._gripper.is_gripping() and self._gripper.get_position() > position):
            return True

        # Release grasped object
        elif (self._gripper.is_gripping() and self._gripper.get_position() < position):
            return False

        # Position has been reached
        return fabs(self._gripper.get_position()-position) < self._dead_zone


    def _update_feedback(self, position):
        self._fdbk.position = self._gripper.get_position()
        self._fdbk.effort = self._gripper.get_force()
        self._fdbk.stalled = not self._gripper.is_moving()
        self._fdbk.reached_goal = (fabs(self._gripper.get_position()-position) <
                                  self._dead_zone)
        self._result = self._fdbk
        self._server.publish_feedback(self._fdbk)


    def _on_gripper_action(self, goal):
        position = max(min(goal.command.position, self._max_pos), self._min_pos)

        if self._gripper.has_error():
            rospy.logerr("%s: Gripper error - please restart action server." %
                         (self._action_name,))
            self._server.set_aborted()

        # Reset feedback/result
        self._update_feedback(position)

        # Prepare to send gripper commands
        self._gripper.start()

        # Update commands at 20 Hz and start clock
        control_rate = rospy.Rate(50.0)
        start_time = rospy.get_time()

        # Continue commanding goal until success or timeout
        while (((rospy.get_time()-start_time) < self._timeout)
                             and not rospy.is_shutdown()):
            if self._server.is_preempt_requested():
                self._gripper.stop()
                rospy.loginfo("%s: Gripper Action Preempted" %
                              (self._action_name,))
                self._server.set_preempted(self._result)
                return

            self._update_feedback(position)

            if self._check_state(position):
                self._server.set_succeeded(self._result)
                return

            self._gripper.set_position(position)
            control_rate.sleep()

        # Gripper failed to achieve goal before timeout/shutdown
        self._gripper.stop()
        if not rospy.is_shutdown():
            rospy.logerr("%s: Gripper Command Not Achieved in Allotted Time" %
                         (self._action_name,))
        self._update_feedback(position)
        self._server.set_aborted(self._result)


def main():
    rospy.init_node('sdk_sawyer_gripper_action_server')
    rospy.loginfo("Initializing node... ")
    gripper_action_server = GripperActionServer()
    rospy.loginfo("Gripper action server initialized")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
