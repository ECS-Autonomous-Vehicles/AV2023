#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def pose_callback(pose_msg):
    # Create an Odometry message
    odom_msg = Odometry()
    
    # Fill the header
    odom_msg.header = pose_msg.header
    
    # Fill the pose information
    odom_msg.pose = pose_msg.pose
    
    # Set the twist information to zero (assuming no velocity information)
    odom_msg.twist.twist.linear.x = 0.0
    odom_msg.twist.twist.linear.y = 0.0
    odom_msg.twist.twist.linear.z = 0.0
    odom_msg.twist.twist.angular.x = 0.0
    odom_msg.twist.twist.angular.y = 0.0
    odom_msg.twist.twist.angular.z = 0.0

    # Publish the Odometry message
    odom_pub.publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('pose_to_odometry', anonymous=True)
    
    # Subscribe to the PoseWithCovarianceStamped topic
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, pose_callback)
    
    # Advertise the Odometry topic
    odom_pub = rospy.Publisher('/odometry_lulus', Odometry, queue_size=10)
    
    # Keep the node running
    rospy.spin()

