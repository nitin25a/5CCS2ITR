#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # New Imports
from delivery_bot.msg import (
    GoToLocationAction, GoToLocationResult,
    LookAtAction, LookAtResult
)
from delivery_bot.srv import GetLocation


class MotionActionServer:
    def __init__(self):
        rospy.init_node("motion_action_server")

        # --- Publishers / Subscribers ---
        # NOTE: We are replacing the simple publisher with an Action Client for move_base
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.current_pose = Pose2D()

        # --- Service Client ---
        rospy.wait_for_service("get_location")
        self.get_location = rospy.ServiceProxy("get_location", GetLocation)

        # --- move_base Action Client ---
        # This client is used to send navigation goals to the ROS move_base node
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("[Action] Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("[Action] Connected to move_base server.")

        # --- Custom Action Servers ---
        self.goto_server = actionlib.SimpleActionServer(
            "goto_location", GoToLocationAction, self.execute_goto, auto_start=False
        )
        self.lookat_server = actionlib.SimpleActionServer(
            "look_at", LookAtAction, self.execute_lookat, auto_start=False
        )

        self.goto_server.start()
        self.lookat_server.start()

        rospy.loginfo("[Action] MotionActionServer running and ready for goals.")
        rospy.spin()

    # --- Callbacks ---
    def odom_callback(self, msg):
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        # Note: You might want to update the orientation here too if you use it.

    # --- Actions ---
    def execute_goto(self, goal):
        rospy.loginfo(f"[Action] Received request: Go to {goal.location_name}")

        # Check for preemption request
        if self.goto_server.is_preempt_requested():
            self.move_base_client.cancel_goal()
            self.goto_server.set_preempted()
            rospy.loginfo("[Action] GoToLocation preempted.")
            return

        # 1. Get target coordinates from the service
        try:
            resp = self.get_location(goal.location_name)
        except rospy.ServiceException as e:
            rospy.logerr(f"[Action] Failed to call get_location: {e}")
            self.goto_server.set_aborted(GoToLocationResult(success=False))
            return

        target = resp.pose
        rospy.loginfo(f"[Action] Navigating to {goal.location_name} at ({target.x:.2f}, {target.y:.2f})")

        # 2. Create the MoveBaseGoal
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.header.frame_id = "map"

        mb_goal.target_pose.pose.position.x = target.x
        mb_goal.target_pose.pose.position.y = target.y

        quat = quaternion_from_euler(0, 0, target.theta)
        mb_goal.target_pose.pose.orientation = Quaternion(*quat)

        # 3. Send goal to move_base and wait for result
        self.move_base_client.send_goal(mb_goal)
        rospy.loginfo(f"[Action] Sent goal to move_base. Waiting for result...")

        # Wait until navigation completes or preemption occurs
        self.move_base_client.wait_for_result()

        state = self.move_base_client.get_state()

        # Check if the goal succeeded (state 3 is SUCCEEDED)
        if state == actionlib.GoalStatus.SUCCEEDED:
            result = GoToLocationResult(success=True)
            self.goto_server.set_succeeded(result)
            rospy.loginfo(f"[Action] Successfully reached {goal.location_name}.")
        else:
            rospy.logerr(f"[Action] Navigation failed or aborted. Final state: {state}")
            result = GoToLocationResult(success=False)
            self.goto_server.set_aborted(result)

    def execute_lookat(self, goal):
        # NOTE: This is a placeholder as you did not implement actual head/base movement
        # to 'look at' the target. It just succeeds immediately.
        rospy.loginfo(f"[Action] Received LookAt target: ({goal.target.x:.2f}, {goal.target.y:.2f})")

        # In a real robot, you would publish a command here to turn the base or the head
        # For now, we simulate success
        rospy.sleep(3)

        result = LookAtResult(success=True)
        self.lookat_server.set_succeeded(result)
        rospy.loginfo("[Action] Finished LookAt action (simulated success).")


if __name__ == "__main__":
    MotionActionServer()
