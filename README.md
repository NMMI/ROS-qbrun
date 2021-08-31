# ROS-qbrun
Simple ROS interface to easily run a chain of qbmoves with the official qbRobotics ROS nodes

This package is used to communicate with a chain of N real qbmove deveices, enabling the sending of commands and reading of sensors measurement. The package exploits the official ROS nodes developed by qbRobotics company.

## How to use it
Download the folder and copy it in your workspace. 

Make sure to have installed also ALL the official qbRobotics [ROS nodes](https://bitbucket.org/%7B976d9f72-c0c3-4fb6-b24d-20519d66e0c9%7D/?search=ROS&sort=-updated_on).

## Configure your qbMove devices
To define the ID values and the name of your robot, edit the following lines of the configuration file inside the *config* folder.
	```yaml
	## Namespace of the robot
	namespace: "your_robot"

	## Cubes parameters of the chain
	IDs: [1, 2]          # cubes identifiers

	## Time parameters (in seconds)
	T_sample: 0.01       # sample time

	## Enable command passed in rad (if true you can command the motors in radians, if false in ticks)
	flag_rad: true
	```

## How to launch it
After connecting the USB cable of your qbmoves chain, and setting the value inside the configuration file, type on the terminal
	```launch
	roslaunch qbrun-ros run_qb.launch
	```

Your chain of qbmove devices will be activated and you will be able to command the motor signals (via array of radiants or ticks, depending on the previous *flag_red* value) through the following topics
	```yaml
	/your_robot/reference_1
	/your_robot/reference_2
	```
The measured signals from the position encoders will be available through the following topics
	```yaml
	/your_robot/motor_1_state		# for motor 1
	/your_robot/motor_2_state		# for motor 2
	/your_robot/robot_state			# for the link
	```

## NOTE
The nomenclature of the topics of this package is the same of the one used in the [ROS-Gazebo-compliant-actuators-plugin](https://github.com/NMMI/ROS-Gazebo-compliant-actuators-plugin).