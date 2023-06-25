#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from ros_clients.msg import GeneralizedForce
from math import pow, atan2, sqrt

class BoatController:
    def _init_(self):
        rospy.init_node('boat_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/force_control', GeneralizedForce, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/nav/pose', PoseStamped, self.update_pose)

        self.pose = PoseStamped()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.pose.position.x - self.pose.pose.position.x), 2) +
                    pow((goal_pose.pose.position.y - self.pose.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.pose.position.y - self.pose.pose.position.y,
                     goal_pose.pose.position.x - self.pose.pose.position.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.pose.orientation.z)

    def move2goal(self):
        goal_pose = PoseStamped()

        goal_pose.pose.position.x = float(input("Set your x goal: "))
        goal_pose.pose.position.y = float(input("Set your y goal: "))

        distance_tolerance = float(input("Set your tolerance: "))

        vel_msg = GeneralizedForce()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.x = self.linear_vel(goal_pose)
            vel_msg.y = 0
            vel_msg.z = 0
            vel_msg.k = 0
            vel_msg.m = 0
            vel_msg.n = self.angular_vel(goal_pose)

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.x = 0
        vel_msg.n = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()

if _name_ == '_main_':
    try:
        boat_controller = BoatController()
        boat_controller.move2goal()
    except rospy.ROSInterruptException:
        pass