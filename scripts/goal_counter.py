#!/usr/bin/env python3
"""
.. module:: goal_counter
    :platform: Unix
    :synopsis: Node B
.. moduleauthor:: Miguel Angel Sempere msemperevicente@gmail.com

Node that implements the service *goal_count*, which, when called, prints 
the number of goals reached and cancelled. It uses the information published on 
the topic */reaching_goal/feedback* to keep track of the reached and cancelled goals.

Subscribes to:
    /reaching_goal/feedback
    
Publishes to:
	None

Services: 
	goal_count

"""

# node B 
# when called, prints the number of goals reached and cancelled

import rospy
from assignment_2_2022.srv import GoalCount, GoalCountResponse
from assignment_2_2022.msg import PlanningActionFeedback


# global variables to count how many goals have been reached and canceled
goals_reached = 0
goals_cancelled = 0

def check_feedback(data):
	"""
	Callback function from the topic */reaching_goal/feedback*. 
	It reads the message of the feedback status and increments the 
	number of cancelled and reached goals depending on its value. 
	It also prints an information message with the modified variable.

	"""	

	global goals_reached, goals_cancelled
	# check the feedback message 
	if data.feedback.stat == "Target cancelled!":
		goals_cancelled += 1
		print("\nCancelled goal: {} ".format(goals_cancelled))
	elif data.feedback.stat == "Target reached!":
		goals_reached += 1
		print("\nReached goal: {} ".format(goals_reached))
	   
def handle_goal_tracker(req):
	"""
	Callback function from the service *goal_count* that prints 
	the number of goals reached and cancelled. These values are also 
	retuned as a server response message of the type 
	``assignment_2_2022::GoalCountResponse``.

	"""	

	global goals_reached, goals_cancelled
	# create variable of the custom service
	data = GoalCountResponse()
	data.reached = goals_reached 
	data.cancelled = goals_cancelled
	# print reached and cancelled goals
	print("\nGoals reached: ", goals_reached, "\nGoals canceled: ", goals_cancelled)
	
	return data
	


def main():
	"""
	Main.

	"""	
	rospy.init_node('goal_counter')
	# create a service that publishes the goal position of the robot
	srv = rospy.Service('goal_count', GoalCount, handle_goal_tracker)
	# subscribe the feedback of the goal to check which message is written
	rospy.Subscriber("/reaching_goal/feedback", PlanningActionFeedback, check_feedback)

	rospy.spin()


if __name__ == '__main__':
	main()

