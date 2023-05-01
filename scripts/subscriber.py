#!/usr/bin/env python3
"""
.. module:: subscriber
    :platform: Unix
    :synopsis: Node C
.. moduleauthor:: Miguel Angel Sempere msemperevicente@gmail.com

Node that subscribes to the topics *robot_data* and *goal_message*, 
and then prints the remaining distance of the robot to the goal 
and the robots average linear and angular speeds. 
When a new goal is received, it starts printing the information 
at a rate established by the parameter *rate_node_c*, 
which can be modified in launch file. If the goal has already been reached, 
the program keeps waiting for a new one.

Subscribes to:
    /robot_data
    /goal_message
    
Publishes to:
	None

"""

import rospy
import math
from assignment_2_2022.msg import RobotData
from geometry_msgs.msg import Point


# Initialize global variables for distance and average velocity
distance = 0
average_linear = 0
average_angular = 0
total_vel_x = 0
total_vel_z = 0
count = 1
goal_reached = 0
current_position = Point()
goal_position = Point()

def robot_callback(robot_msg): # extract position and velocity data
	"""
	Callback function from the topic */robot_data*. 
	It extracts the current position and velocity data from 
	the received message of type ``assignment_2_2022::RobotData``.

	"""	
	global current_position, total_vel_x, total_vel_z
	
	current_position.x = robot_msg.x
	current_position.y = robot_msg.y
	total_vel_x += robot_msg.vel_x
	total_vel_z += robot_msg.vel_z
	

	
def goal_callback(goal_msg):
	"""
	Callback function from the topic */goal_message*. 
	It extracts the coordinates of the received message of type 
	``geometry_msgs::Point`` and prints the new goal position on the 
	command window. At the same time, the computation of average speeds 
	is reset every time a new goal is received.
	
	"""		
	global goal_position, count, total_vel_x, total_vel_z, goal_reached

	count = 1
	total_vel_x = 0
	total_vel_z = 0
	goal_reached = 0
	
	goal_position.x = goal_msg.x
	goal_position.y = goal_msg.y
	
	print("Received new goal position: [", goal_msg.x, ", ", goal_msg.y, "]")


def main():
	"""
	Main.

	"""		
	global distance, average_linear, average_angular, current_position, goal_position, count, total_vel_x, total_vel_z, goal_reached
	
	# Initialize a ROS node and subscribers to the "state_message"
	rospy.init_node('subscriber')
	rospy.Subscriber('robot_data', RobotData, robot_callback)
	rospy.Subscriber('goal_message', Point, goal_callback)
	
	# Get the rate of execution of this node from the ROS parameter server (frequency in Hz)
	rate_node_c = rospy.get_param('rate_node_c')
	rate = rospy.Rate(rate_node_c)

	while not rospy.is_shutdown():
		
		# increment count of obtained values
		count += 1
		
		# compute the distance from the current position to the goal position
		distance = math.dist([current_position.x, current_position.y], [goal_position.x, goal_position.y])
		
		# the computation of average speeds is reset once a the goal is reached
		if distance < 0.51 and goal_reached == 0: # the error precision is set to 0.5
			count = 1
			total_vel_x = 0
			total_vel_z = 0
			goal_reached = 1
			print('Goal reached!\n')
		
		# compute average speeds
		average_linear = total_vel_x / count 
		average_angular = total_vel_z / count
		
		if goal_reached == 0:
			# print values
			print('\nRemaining distance:', round(distance,2))
			print('Average linear velocity:', round(average_linear,2))
			print('Average angular velocity: ', round(average_angular,2))
		else:
			print('Waiting for a new goal...')
		
		rate.sleep()



if __name__ == "__main__":
	main()
