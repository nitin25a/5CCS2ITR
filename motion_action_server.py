#!/usr/bin/env python3
import rospy
import actionlib
from math import atan2, hypot, sin, cos
from geometry_msgs.msg import Twist, Pose2D, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from delivery_bot.msg import (
    GoToLocationAction, GoToLocationFeedback, GoToLocationResult,
    LookAtAction, LookAtFeedback, LookAtResult
)
from delivery_bot.srv import GetLocation

class MotionActionServer:
    def __init__(self):
        rospy.init_node('motion_action_server')

        # Publisher to move the robot
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Subscribe to odometry to track robot position
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_pose = Pose2D()

        # Connect to the location service
        rospy.wait_for_service('get_location')
        self.get_location = rospy.ServiceProxy('get_location', GetLocation)

        # Create two action servers
        self.goto_server = actionlib.SimpleActionServer('goto_location', GoToLocationAction, self.execute_goto, False)
        self.lookat_server = actionlib.SimpleActionServer('look_at', LookAtAction, self.execute_lookat, False)

        self.goto_server.start()
        self.lookat_server.start()
        rospy.loginfo("Action servers started.")
        rospy.spin()

    def odom_callback(self, msg):
        """Extracts x, y, yaw from odometry."""
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        self.current_pose.theta = yaw

    def execute_goto(self, goal):
        """Handles GoToLocation action: moves robot to a named room."""
        rate = rospy.Rate(10)
        resp = self.get_location(goal.location_name)
        target = resp.pose
        rospy.loginfo(f"[Action] Navigating to {goal.location_name} ({target.x:.2f}, {target.y:.2f})")

        vel = Twist()
        feedback = GoToLocationFeedback()

        while not rospy.is_shutdown():
            dx = target.x - self.current_pose.x
            dy = target.y - self.current_pose.y
            distance = hypot(dx, dy)

            # Stop if close enough
            if distance < 0.3:
                break

            angle_to_target = atan2(dy, dx)
            angle_error = angle_to_target - self.current_pose.theta
            angle_error = atan2(sin(angle_error), cos(angle_error))

            vel.linear.x = 0.5
            vel.angular.z = 0.8 * angle_error
            self.cmd_pub.publish(vel)

            feedback.current_pose = self.current_pose
            self.goto_server.publish_feedback(feedback)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        result = GoToLocationResult(success=True)
        self.goto_server.set_succeeded(result)
        rospy.loginfo(f"[Action] Reached {goal.location_name}")

    def execute_lookat(self, goal):
        """Handles LookAt action: rotates robot to face a target point."""
        rate = rospy.Rate(10)
        vel = Twist()
        feedback = LookAtFeedback()
        result = LookAtResult(success=True)
        target = goal.target

        rospy.loginfo(f"[Action] Looking at target ({target.x:.2f}, {target.y:.2f})")

        while not rospy.is_shutdown():
            dx = target.x - self.current_pose.x
            dy = target.y - self.current_pose.y
            desired_yaw = atan2(dy, dx)
            yaw_error = desired_yaw - self.current_pose.theta
            yaw_error = atan2(sin(yaw_error), cos(yaw_error))

            if abs(yaw_error) < 0.05:
                break

            vel.angular.z = 0.5 * yaw_error
            self.cmd_pub.publish(vel)
            feedback.current_yaw = self.current_pose.theta
            self.lookat_server.publish_feedback(feedback)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        self.lookat_server.set_succeeded(result)
        rospy.loginfo("[Action] Finished LookAt action.")

if __name__ == '__main__':
    MotionActionServer()