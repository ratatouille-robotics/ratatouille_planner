#! /usr/bin/env python

import rospy

import actionlib

import ratatouille_planner.msg


class RatatouilleAction(object):
    # create messages that are used to publish feedback/result
    _feedback = ratatouille_planner.msg.RecipeRequestFeedback()
    _result = ratatouille_planner.msg.RecipeRequestResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ratatouille_planner.msg.RecipeRequestAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        rospy.loginfo("Server started")
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo("%s: Executing" % (self._action_name))

        # start executing the action
        for i in range(1, 10):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._as.set_preempted()
                success = False
                break

            self._feedback.percent_complete = i * 10
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

        if success:
            self._result.status = "OK"
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == "__main__":
    rospy.init_node("RecipeRequest")
    server = RatatouilleAction(rospy.get_name())
    rospy.spin()
