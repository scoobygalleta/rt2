#!/usr/bin/env python3

# node B 
# when called, prints the number of goals reached and cancelled

import rospy
from assignment_2_2022.srv import GoalCount, GoalCountResponse
from assignment_2_2022.msg import PlanningActionFeedback


# global variables to count how many goals have been reached and canceled
goals_reached = 0
goals_cancelled = 0

def check_feedback(data):
	
	global goals_reached, goals_cancelled
	# check the feedback message 
	if data.feedback.stat == "Target cancelled!":
	   goals_cancelled += 1
	   print("\nCancelled goal: {} ".format(goals_cancelled))
	elif data.feedback.stat == "Target reached!":
	   goals_reached += 1
	   print("\nReached goal: {} ".format(goals_reached))
	   
def handle_goal_tracker(req):

	global goals_reached, goals_cancelled
	# create variable of the custom service
	data = GoalCountResponse()
	data.reached = goals_reached 
	data.cancelled = goals_cancelled
	# print reached and cancelled goals
	print("\nGoals reached: ", goals_reached, "\nGoals canceled: ", goals_cancelled)
	
	return data
	


def main():
	rospy.init_node('goal_counter')
	# create a service that publishes the goal position of the robot
	srv = rospy.Service('goal_count', GoalCount, handle_goal_tracker)
	# subscribe the feedback of the goal to check which message is written
	rospy.Subscriber("/reaching_goal/feedback", PlanningActionFeedback, check_feedback)

	rospy.spin()


if __name__ == '__main__':
	main()

