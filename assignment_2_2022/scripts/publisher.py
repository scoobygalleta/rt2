#!/usr/bin/env python3

# node A publisher
# node that publishes the robot position and velocity as a custom message
# (x,y, vel_x, vel_z) by relying on the values published on the topic /odom

import rospy
from nav_msgs.msg import Odometry
from assignment_2_2022.msg import RobotData


def odom_callback(odom_data):
	
    # Extract the position and velocity data from the received message of type Odom
	current_data = RobotData()

	current_data.x = odom_data.pose.pose.position.x
	current_data.y = odom_data.pose.pose.position.y
	current_data.vel_x = odom_data.twist.twist.linear.x
	current_data.vel_z = odom_data.twist.twist.linear.y
	
	pub = rospy.Publisher('robot_data', RobotData, queue_size = 10)
	pub.publish(current_data)
  
  
def main():
    # Initialize the node
    rospy.init_node('publisher')
    # Subscribe to the /odom topic
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback) 
    # Spin the node to keep it running
    rospy.spin()
		
if __name__ == '__main__':
    main()

    
    
         
