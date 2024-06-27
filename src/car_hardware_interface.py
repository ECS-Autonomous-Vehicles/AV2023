#!/usr/bin/env python
import rospy
import math
import cvxpy
import numpy as np
import tf

from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from teb_local_planner.msg import FeedbackMsg
from tf.transformations import euler_from_quaternion

last_count_l = 0
last_count_r = 0


class Hardware_interface:
    def __init__(self):
        # rospy.Subscriber("/right_ticks", Int16, self.encoder_right_callback, queue_size=100)
        # rospy.Subscriber("/left_ticks", Int16, self.encoder_left_callback, queue_size=100)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=100)
        

        self.base_speed = 1500
        self.base_angle = 90
        self.speed = self.base_speed
        self.angle = self.base_angle

        self.ticks_per_meter = 3100 # Original was 2800

    # def encoder_right_callback(self,data):
    #     # Processes right wheel encoder data and calculates distance.
    #     last_count_r
    #     if data != 0 and last_count_r != 0:
    #         right_ticks = data - last_count_r
    #         # Handle encoder overflow/underflow
    #         if right_ticks > 10000:
    #             self.distance_right = (0 - (65535 - last_count_r)) / self.ticks_per_meter
    #         elif right_ticks < -10000:
    #             right_ticks = 65535 - right_ticks

    #         self.distance_right = right_ticks / self.ticks_per_meter
    #     else:
    #         self.distance_right = 0  # Set distance to 0 if no valid data received
    #     # Update last count for next calculation
    #     last_count_r = data

    #     return self.distance_right

    # def encoder_left_callback(self,data):
    #     # Processes right wheel encoder data and calculates distance.
    #     last_count_l
    #     if data != 0 and last_count_l != 0:
    #         left_ticks = data - last_count_l
    #         # Handle encoder overflow/underflow
    #         if left_ticks > 10000:
    #             self.distance_left = (0 - (65535 - last_count_l)) / self.ticks_per_meter
    #         elif left_ticks < -10000:
    #             left_ticks = 65535 - left_ticks

    #         self.distance_left = left_ticks / self.ticks_per_meter
    #     else:
    #         self.distance_left = 0  # Set distance to 0 if no valid data received
    #     # Update last count for next calculation
    #     last_count_l = data

    #     return self.distance_left
    
    def cmd_vel_callback(self, cmd):
        # Processes cmd_vel data and converts it to motor and servo control signals.
        # cmd.linear.x is the desired linear velocity (m/s)
        # cmd.angular.z is the steering angle (rad)

        # Calculate the desired speed of each wheel
        self.speed = self.base_speed - np.round((cmd.linear.x * 666.66667)/5) # max speed motor 0.4 m/s sedangkan dari simulasi max nya 2 m/s
        # Calculate the desired angle of each wheel
        self.angle = self.base_angle - np.round(np.rad2deg(cmd.angular.z))


        # Publish the desired speed and angle to the hardware
    def run(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = self.speed
        cmd_msg.angular.z = self.angle
        print("Motor Speed", self.speed)
        print("Servo Angle", self.angle)
        pub.publish(cmd_msg)


if __name__=='__main__':
    rospy.init_node("hardware_interface")
    rospy.loginfo("Node has been started")
    rate = rospy.Rate(10)
    hardware = Hardware_interface()
    pub = rospy.Publisher("/car/cmd_vel", Twist, queue_size=10 )
    while not rospy.is_shutdown():
        # rospy.spin()
        hardware.run()
        rate.sleep()
