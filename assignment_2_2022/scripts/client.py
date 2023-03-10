#!/usr/bin/env python3

# node A client
# implements an action client, allowing the user to set a target (x, y)
# or to cancel it.
 
import rospy
import time
import actionlib #for action
import actionlib.msg  
import assignment_2_2022.msg
from assignment_2_2022.srv import GoalCount, GoalCountRequest
from geometry_msgs.msg import Point

goals_count = 0

def get_new_goal(): # ask for a new goal point
    
    # create a goal to be sent to the action server.
    goal = assignment_2_2022.msg.PlanningGoal()

    print("Please specify the new goal:")
    # convert the input of the user to float and save it as the goal target pose
    goal.target_pose.pose.position.x = float(input("X : "))
    goal.target_pose.pose.position.y = float(input("Y : "))

    return goal


def main(): 
	
	# time for Gazebo to start
    time.sleep(6) 
    
    # initialize the node
    rospy.init_node('nodeA_client')

    # create the action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)
    
    # publisher for publishing the goal positions in node C
    goal_pub = rospy.Publisher('goal_message', Point, queue_size=10)

    # waits for the action server to start
    client.wait_for_server()
    
    # waits for the service
    rospy.wait_for_service('goal_count')
    counter_service = rospy.ServiceProxy('goal_count', GoalCount)
    
    global goals_count
    
    # loop rate
    rate = rospy.Rate(2)

    # loop
    while not rospy.is_shutdown():

        goal_msg = Point()
  
        # selection menu
        print("\n--------------------- Please select an option ---------------------")
        option = input("(1) New goal || (2) Delete goal || (3) Goal counter || (4) Continue ")
        
        if option == '1':
            # send the goal to the action server
            new_goal = get_new_goal()
            client.send_goal(new_goal)
            # send the goal to node C
            goal_msg.x = new_goal.target_pose.pose.position.x
            goal_msg.y = new_goal.target_pose.pose.position.y
            goal_pub.publish(goal_msg)
            
            goals_count = 1 
        elif option == '2':
            if goals_count != 0:
                # cancel the goal
                client.cancel_goal()
                goals_count = 0
            else:
                 print("There is no goal to be deleted \n")
        elif option == '3':
            request = GoalCountRequest()
            counter_result = counter_service(request)
        elif option == '4':
            continue
        else:
            print("Invalid option selected\n")


        
        rate.sleep()

if __name__ == '__main__':
    main()
