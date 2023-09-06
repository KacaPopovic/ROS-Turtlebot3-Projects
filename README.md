# ROS-Turtlebot3-Projects
Projects in autonomous mobile robotics using Robot Operating System, Gazebo and Turtlebot3

# Project 1 #
**Introduction to ROS**

We've structured this project by creating distinct nodes to handle specific tasks. Each of these nodes corresponds to a particular action that needs to be executed. These actions include loading data into the measurements node, data processing within the processing node, data display in the display node, and an action node responsible for signaling specific events within the data. Communication between these nodes is facilitated through ROS topics, and you can gain a visual understanding of the nodes and topics through the 'graph.png' file provided in the project directory. Additionally, we've incorporated functionality for adding new data, such as temperature readings, using the 'add_new_values' service.

Instructions for Running the ROS Scripts:

1. Begin by opening a terminal on your Linux-based PC and launching the ROS core using the 'roscore' command.
2. Navigate to the 'src' directory of the project using the 'cd' command.
3. Execute the Python scripts using the 'rosrun' command. For example, you can run 'rosrun dz1 action.py'. Remember to open a new terminal window for each script. The recommended order for running the scripts is as follows: 'action.py', 'display.py', 'processing.py', 'measurements.py'.
4. To add new data, such as temperature values, through the service, use the 'rosservice' command. For instance, you can invoke the service like this: 'rosservice call /add_new_values "{new_max: 150.0, new_min: 20.0, new_avg: 45.0}"'.
5. To visualize a graph depicting the average temperatures, employ the 'rqt_plot' command and select the 'top_display' topic.
6. For a graphical representation of the project's node and topic structure, run 'rqt_graph'.

# Project 2 #
**Movement control**

We create node that uses Subcriber to read position of our robot TURTLEBOT3 in Gazebo simulation from his odometry sensor, reading from odom topic, and Publisher to move robot in simulation sending commands to read_command topic. In nodes, we call functions that generate commands we send to robot, depending of the selected mode of movement, that user selects from service.

Instructions for running the scripts using ROS

1. Run terminal on your Linux PC, and run command roscore.
2. To run simulation, run commands export TURTLEBOT3_MODEL=burger and roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch.
3. Use cd command to navigate to src directory of the project.
4. To run Python scripts use command rosrun, e.g. rosrun dz2 main.py.
5. To move robot using service, run command rosservice, e.g. rosservice call /read_command "{mode: "M"}".
6. To control robot in Manuel mode, usinh keyboard, using service set mode to 'M' (e.g. erosservice call /read_command "{mode: "M"}") and run commands export TURTLEBOT3_MODEL=burger and roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch --screen and move it using keys W, A, S, D, X.
7. To control robot in Auto mode, using service set mode to 'A' and set targets (e.g. rosservice call /read_command "{mode: "A", x_target: 0, y_target: 0, theta_target: 0}").
8. To control robot in Auto mode with constat velocity, using service set mode to 'AC' and set targets (e.g. rosservice call /read_command "{mode: "A", x_target: 0, y_target: 0, theta_target: 0}").
