## Time-Controlled Dual Robot Simulator

The time-controlled dual robot simulator is a Python application that uses ROS and Tkinter to control two turtlesim robots. The simulator allows users to command the robots to perform movements in specified patterns (Rectangle or Triangle) either simultaneously or sequentially. Additionally, users can set a countdown timer and schedule the robot movements to repeat at regular intervals.

# Prerequisites
1. ROS Noetic
2. Python 3
3. Turtlesim Package

# Installation
1. Set up the ROS workspace. Place the 'dual_robot_simulator' package into ROS workspace 'src' directory. Then type 'catkin_make' and source the workspace by typing 'source ~/ros_ws/devel/setup.bash'

# Running the Simulator
1. Launch the turtlesim nodes and dual-robot simulator nodes by typing "roslaunch dual_robot_simulator dual_robot_sim.launch".
2. Interact with GUI
   a. A thinker GUI window should appear.
   b. Set the time for the countdown and specify/choose whether you want the robots to move simultaneously (click "move simultaneously?") or sequentially (click "move Sequentially?")
   c. Choose the movement pattern for each robot (if you click the menus, it will show two option 'rectangle' or 'triangle')
   d. If you want to see the repeat operation after 2 minutes, you can choose the "repeat every 2 minutes?" checkbutton on GUI
   e. Press "Start" button to begin the operation. The countdown will start, and the robots will perform the specified movements.

# Features
1. Movement Patterns: Choose between rectangle or triangle movement patterns for each robot
2. Simultaneously/Sequential Movement: Move both the robot simultaneously or sequentially
3. Countdown Timer: Set a countdown before the robot start moving
4. Repeat Operation: Optionally, set the robots to repeat their movements every 2 minutes

# Additional Note
1. the script of this program is located inside script folder which has a name file "dual_robot_sim.py"
2. the launch file is located inside launch folder which has a name file "dual_robot_sim.launch"
3. README file inside the ROS package
4. Video demonstration how to run the program and how to use GUI is located inside video folder
