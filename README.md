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

