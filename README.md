Research Track I - Assignment 2  
Miguel Ángel Sempere Vicente - s5646720 
----------------------

This repository contains the ROS package developed for the second assignment of Reseach Track I: *assignment_2_2022*.


## Building and running the code
The ros package can be built by executing the command `catkin_make` within the root directory of the ROS workspace.

It may be the case that the scripts are not executable, to make these Python scripts executable, it is enough to execute the following command inside the `scripts` folder:
```console
chmod +x *.py
```

Finally, in order to run the simulation, a launch file has been created. Its name is *assignment_2_2022.launch*. So, you can run the whole simulation by executing the following command:
```console
roslaunch assignment_2_2022 assignment2.launch
```

The following windows will pop up:
- **Gazebo**, with the simulation of the robot in the environment.
- **RViz**, to also visualize the robot.
- All the scripts will run in the same terminal window used to do the roslaunch, the script *subscriber.py* should be executed separately on another terminal so that the menu displayed by *client.py* stays readable for the user.

## Contents of the folder
The package *assignment_2_2022* is organized as follows:
- The `scripts` folder, which contains the Python scripts corresponding to the three nodes.
- The `msg` folder, which contains the definition of the custom message *RobotData*.
- The `srv` folder, which contains the definition of the custom service *GoalCount*.
- The `launch` folder, that contains the launch file (*assignment2.launch*).
- The *CMakeLists.txt* file.
- The package manifest (*package.xml*).


## Description of the scripts
- **client.py**. nodeA client:  implements an action client for the action service *reaching_goal*, allowing the user to set a new target (x, y), cancel it, or call the *goal_counter.py* script (nodeB) to display the number of cancelled and reached goals so far. The program also keeps track locally of the entered  goals and does not let to delete a goal if no goal is currently active.
- **publisher.py**. nodeA publisher: it subscribes and gets values from the topic *odom*, and then publishes on the topic *robot_data* the robot's position and velocities using the custom message *RobotData*, which containes the fields *x*, *y*, *vel_x* and *vel_z*, 
- **goal_counter.py**. nodeB: implements the service *goal_count*, which, when called, prints and returns the number of goals reached and cancelled.
- **subscriber.py**. nodeC: it subscribes to the topics *robot_data* and *goal_message*, and then prints the remaining distance of the robot to the goal and the robots average linear and angular speeds.  When a new goal is received, it starts printing the information at a rate established by the the variable *rate*, which can be modified in the code. If the goal has already been reached, the program keeps waiting for a new one.

## Pseudo-code of *client.py*
```console
FUNCTION "get_new_goal":
	
	Declare a goal to be sent to the action server (PlanningGoal type)
	Ask for the user to input the goal coordinates
	Convert the input of the user to float and save it as the goal target pose
    Return goal
	

FUNCTION "main":
	Sleep some time for Gazebo to start

	Initialize the node "client"
	
	Create a simple action client of the action service '/reaching_goal', type 'PlanningAction'
	Wait for the action server to start listening for goals
	
	Declare the publisher on topic "goal_message" for publishing the goal positions (type "Point") in node C
	
	Wait for "goal_count" service (custom service type GoalCount)

	Stablish loop rate

	while this node is running:
		Call 
		Display the user menu and save the input (option)

		if option == new goal
			Save the returned goal from function "get_new_goal"
			Send the goal to the action server
			Send the goal to node C (for the computation of the distance to goal)
			goals_count local variable set to 1
		elseif option == delete goal
			
			if goals_count not 0
				Call cancel_goal on the client		
			else
				Print no goal to be deleted
		elseif option == goal count
			Send request GoalCountRequest()
			Get back the counter result from goal_count
		elseif option == continue
			Continue
		else
			Print invalid option selected

		Sleep for loop rate

```
## Pseudo-code of *publisher.py*

```console
FUNCTION "odom_callback":
	Declare a message of the custom type "RobotData"
	Extract the position and velocity data from the received message of type Odom
	Fill the four fields of the "RobotData" message
	Publish the message via the topic "robot_data"

FUNCTION "main":
	Initialize the node "publisher"
	Subscribe to the topic "odom" (type "Odometry") setting "odom_callback" as callback function
	Spin the node to keep it running
```