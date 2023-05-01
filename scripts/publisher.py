#!/usr/bin/env python3
"""
.. module:: publisher
    :platform: Unix
    :synopsis: Node A Publisher
.. moduleauthor:: Miguel Angel Sempere msemperevicente@gmail.com

Node that publishes the robot position and velocity as a custom message
(x, y, vel_x, vel_z) by relying on the values published on the topic /odom.

Subscribes to:
    /odom

Publishes to:
    /robot_data

"""

import rospy
from nav_msgs.msg import Odometry
from assignment_2_2022.msg import RobotData


def odom_callback(odom_data):
    """
    Callback function from the topic */odom*. 
    It extracts the position and velocity data from 
    the received message of type ``nav_msgs::Odometry`` and 
    publishes another messsage of type 
    ``assignment_2_2022::RobotData`` on the topic */robot_data*.

    """
    current_data = RobotData()

    current_data.x = odom_data.pose.pose.position.x
    current_data.y = odom_data.pose.pose.position.y
    current_data.vel_x = odom_data.twist.twist.linear.x
    current_data.vel_z = odom_data.twist.twist.linear.y
	
    pub = rospy.Publisher('robot_data', RobotData, queue_size = 10)
    pub.publish(current_data)
  
  
def main():
    """
    This function initializes the node *publisher* and
    subscribes to the topic */odom* assigning odom_callback 
    as the callback function.
    """    
    # Initialize the node
    rospy.init_node('publisher')
    # Subscribe to the /odom topic
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback) 
    # Spin the node to keep it running
    rospy.spin()
		
if __name__ == '__main__':
    main()

    
    
         
