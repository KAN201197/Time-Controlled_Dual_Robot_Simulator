#!/usr/bin/env python3

from tkinter import ttk
import tkinter as tk
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import threading
import time

class DualRobotSimulator:
    def __init__(self):
        # Initialize variables
        self.remaining_time = 0  # Keep track of the remaining time for the countdown (from user) and repeat operation
        self.repeat_time = 0
        self.robot1_lock = threading.Lock() # Ensure thread safety and not conflicted by other treads
        self.robot2_lock = threading.Lock()
        self.robot1_moving = False # Flag indicate robot status
        self.robot2_moving = False
        self.repeat_timer = None # Used with 'threading.Timer' to manage repeated operations
        self.repeat_countdown_thread = None # Store the previous countdown thread for repeat timer

        # Initialize the Tkinter root Window
        self.root = tk.Tk()
        self.root.title("Time-Controlled Dual Robot Simulator")

        # Set up the user interface
        self.create_ui()

        # Initialize ROS Node
        rospy.init_node('dual_robot_controller', anonymous=True)
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("ROS node initialized")

        # Initialize pose data for each robot
        self.robot1_pose = None
        self.robot2_pose = None

        # Subscribe to pose topics for both robots
        self.robot1_pose_sub = rospy.Subscriber('/turtle1/turtle1/pose', Pose, self.update_robot1_pose)
        self.robot2_pose_sub = rospy.Subscriber('/turtle2/turtle1/pose', Pose, self.update_robot2_pose)

        # Start the Thinker main loop
        self.root.mainloop()
    
    """ The function which handles shutdown of the ROS Node """
    def shutdown_hook(self):
        rospy.loginfo("Shutting down ROS Node...")
        if self.repeat_timer:
            self.repeat_timer.cancel() # prevent from executing scheduled function (repeat operation) after the ROS Node died

    """ The function used for creating the user interface elements """
    def create_ui(self):
        style = ttk.Style()
        # Configure styles for different widgets
        style.configure("TLabel", font=("Helvetica", 16))
        style.configure("TButton", font=("Helvetica", 16))
        style.configure("TCheckbutton", font=("Helvetica", 16))
        style.configure("TEntry", font=("Helvetica", 16))
        style.configure("TMenubutton", font=("Helvetica", 16))

        # store padding value to add space around widgets
        padding = {'padx': 20, 'pady': 20}

        # Create and place widgets for time input
        ttk.Label(self.root, text="Set Time (Seconds):").grid(column=0, row=0, **padding)
        self.time_entry = ttk.Entry(self.root, font=("Helvetica", 16))
        self.time_entry.grid(column=1, row=0, **padding)

        # Initialize boolean variable for control options (simultaneous, sequential, and repeat)
        self.simultaneous_var = tk.BooleanVar()
        self.sequential_var = tk.BooleanVar()
        self.repeat_var = tk.BooleanVar()

        # Intiliaze string variable for control options (movement option)
        self.robot1_movement_var = tk.StringVar(value="Rectangle")
        self.robot2_movement_var = tk.StringVar(value="Triangle")

        # Checkbox options for movement type (simultaneous or sequential) and repeat operation
        ttk.Checkbutton(self.root, text="Move Simultaneously?", variable=self.simultaneous_var, command=self.check_buttons).grid(column=0, row=2, **padding)
        ttk.Checkbutton(self.root, text="Move Sequentially?", variable=self.sequential_var, command=self.check_buttons).grid(column=1, row=2, **padding)
        ttk.Checkbutton(self.root, text="Repeat every 2 minutes?", variable=self.repeat_var).grid(column=0, row=4, columnspan=2, **padding)

        # Option menus for robot 1 movement type (Rectangle or Triangle)
        ttk.Label(self.root, text="Robot1 Movement:").grid(column=0, row=6, **padding)
        ttk.OptionMenu(self.root, self.robot1_movement_var, "Rectangle", "Rectangle", "Triangle").grid(column=1, row=6, **padding)

        # Option menus for robot 2 movement type (Rectangle or Triangle)
        ttk.Label(self.root, text="Robot2 Movement:").grid(column=0, row=7, **padding)
        ttk.OptionMenu(self.root, self.robot2_movement_var, "Triangle", "Rectangle", "Triangle").grid(column=1, row=7, **padding)

        # Configure start button to begin simulation
        self.start_button = ttk.Button(self.root, text="Start", command=self.start_execution)
        self.start_button.grid(column=0, row=8, columnspan=2, **padding)

        # Label to show remaining time for set time by user (delay time) and repeat operation in real-time
        self.remaining_time_label = ttk.Label(self.root, text="Remaining Time: N/A Seconds", font=("Helvetica", 16))
        self.remaining_time_label.grid(column=0, row=9, columnspan=2, **padding)

        # Label to show remaining time for set time by user (delay time) and repeat operation in real-time
        self.repeat_time_label = ttk.Label(self.root, text="Repeat in: N/A Seconds", font=("Helvetica", 16))
        self.repeat_time_label.grid(column=0, row=10, columnspan=2, **padding)

        # Label to show the robot 1 position in real-time
        self.robot1_position_label = ttk.Label(self.root, text="Robot1 Position: (N/A, N/A)")
        self.robot1_position_label.grid(column=0, row=11, columnspan=2, **padding)

        # Label to show the robot 2 position in real-time
        self.robot2_position_label = ttk.Label(self.root, text="Robot2 Position: (N/A, N/A)")
        self.robot2_position_label.grid(column=0, row=12, columnspan=2, **padding)
    
    """ Callback function to update the position of the robot 1 """
    def update_robot1_pose(self, msg):
        self.robot1_pose = msg
        self.update_robot_position_labels()

    """ Callback function to update the position of the robot 2 """
    def update_robot2_pose(self, msg):
        self.robot2_pose = msg
        self.update_robot_position_labels()

    """ Function used to updates the UI labels with the latest positions for both robots """
    def update_robot_position_labels(self):
        if self.robot1_pose:
            self.robot1_position_label.config(text=f"Robot1 Position: ({self.robot1_pose.x:.2f}, {self.robot1_pose.y:.2f})")
        if self.robot2_pose:
            self.robot2_position_label.config(text=f"Robot2 Position: ({self.robot2_pose.x:.2f}, {self.robot2_pose.y:.2f})")

    """ Function which ensure only one of the movement options (simultaneous or sequential) can be seletected at a time """
    def check_buttons(self):
        if self.simultaneous_var.get():
            self.sequential_var.set(False)
        elif self.sequential_var.get():
            self.simultaneous_var.set(False)
        
        # Cancel repeat timer if repeat option is deselected
        if not self.repeat_var.get() and self.repeat_timer:
            self.repeat_timer.cancel()

    """ Function used to move the robot in a rectangle pattern 
        @param pub The robot publisher variable
        @param lock The treading lock
    """
    def move_in_rectangle(self, pub, lock):
        with lock:
            move_cmd = Twist()
            for _ in range(4):
                # Move forward
                move_cmd.linear.x = 2.0
                move_cmd.angular.z = 0.0
                pub.publish(move_cmd)
                rospy.sleep(2)

                # Turn
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 1.57  # Rotate 90 degrees
                pub.publish(move_cmd)
                rospy.sleep(1)

            # Stop the robot after completing all movements
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)

    """ Function used to move the robot in a triangle pattern 
        @param pub The robot publisher variable
        @param lock The treading lock
    """
    def move_in_triangle(self, pub, lock):
        with lock:
            move_cmd = Twist()
            for _ in range(3):
                # Move forward
                move_cmd.linear.x = 2.0
                move_cmd.angular.z = 0.0
                pub.publish(move_cmd)
                rospy.sleep(2)

                # Turn
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 2.09  # Rotate 120 degrees
                pub.publish(move_cmd)
                rospy.sleep(1)

            # Stop the robot after completing all movements
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)

    """ The function to start the robot operation based on user settings 
        @param simultaneous The simultaneous variable option in GUI (True or False)
        @param sequential The sequential variable option in GUI (True or False)
        @param repeat The repeat variable option in GUI (True or False)
    """
    def start_robot_operation(self, simultaneous, sequential, repeat):
        rospy.loginfo("Starting robot operation...")
        robot_pub1 = rospy.Publisher('/turtle1/turtle1/cmd_vel', Twist, queue_size=10)
        robot_pub2 = rospy.Publisher('/turtle2/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)  # Allow time for publishers to be set up

        # Get the each robot movement type (Rectangular or Triangle)
        robot1_movement = self.robot1_movement_var.get()
        robot2_movement = self.robot2_movement_var.get()

        if simultaneous:
            rospy.loginfo("Moving both robots simultaneously...")
            if not self.robot1_moving and not self.robot2_moving: # Simultaneous operation only executed if both of the robot is not moving
                self.robot1_moving = True
                self.robot2_moving = True

                # Start threads for both robots
                threading.Thread(target=self.execute_movement, args=(robot_pub1, self.robot1_lock, robot1_movement, 1)).start()
                threading.Thread(target=self.execute_movement, args=(robot_pub2, self.robot2_lock, robot2_movement, 2)).start()
                self.wait_for_both_robots()

            rospy.loginfo("Moving both robots simultaneously finish...")

        elif sequential:
            rospy.loginfo("Moving robots sequentially...")
            if not self.robot1_moving: # Sequential operation executed only for robot 1 and next robot 2 if is not moving
                self.robot1_moving = True
                threading.Thread(target=self.execute_movement, args=(robot_pub1, self.robot1_lock, robot1_movement, 1)).start()

                # Ensure robot 1 movement is completed before starting robot 2 movement
                threading.Thread(target=self.check_and_start_next_robot, args=(robot_pub2, robot2_movement)).start()
                self.wait_for_both_robots()

            elif not self.robot2_moving: # Sequential operation executed only for robot 2 if is not moving
                self.robot2_moving = True
                threading.Thread(target=self.execute_movement, args=(robot_pub2, self.robot2_lock, robot2_movement, 2)).start()
                self.wait_for_both_robots()

            rospy.loginfo("Moving robots sequentially finish...")
        
        if repeat:
            rospy.loginfo("Setting up repeat operation...")
            self.cancel_previous_repeat_countdown_thread() # Cancel the previous repeat countdown thread
            self.repeat_time = 120 # Set repeat time to 120 seconds following the repeat operation
            self.repeat_countdown_thread = threading.Thread(target=self.countdown_timer_repeat_time, args=(self.update_repeat_time_label,))
            self.repeat_countdown_thread.start()

            # Cancel any existing repeat timer to prevent overlapping repeat operations
            if self.repeat_timer:
                self.repeat_timer.cancel()

            # Set up a timer to repeat the operation every 2 minutes
            self.repeat_timer = threading.Timer(120, self.start_robot_operation, args=(simultaneous, sequential, repeat))
            self.repeat_timer.start()
            rospy.loginfo("Repeat operation start...")

    """ Function used to execute the specified movement pattern (Rectangular or Triangle) for specific number of robot (1 or 2)
        @param pub The robot publisher variable
        @param lock The treading lock
        @param movement pattern of movement (rectangle or triangle)
        @param robot_number specific robot number (1 or 2)
    """
    def execute_movement(self, pub, lock, movement, robot_number):
        rospy.loginfo(f"Robot{robot_number} is moving in {movement} pattern")
        if movement == "Rectangle":
            self.move_in_rectangle(pub, lock)
        elif movement == "Triangle":
            self.move_in_triangle(pub, lock)

        # Mark robot as not moving after completing its movement
        if robot_number == 1:
            self.robot1_moving = False
        else:
            self.robot2_moving = False

    """ Function for waiting Robot 1 to finish its movement then can start to execute the Robot 2 Movement (Required in Sequential Movement)
        @param pub The robot publisher variable
        @param movement pattern of movement (rectangle or triangle)
    """
    def check_and_start_next_robot(self, pub, movement):
        while self.robot1_moving:
            rospy.sleep(1)
        if not self.robot2_moving:
            self.robot2_moving = True
            self.execute_movement(pub, self.robot2_lock, movement, 2)

    """ Function used for count down the time and updates the UI label for remaining time (set time of delay)
        @param update_label_func function use to update the UI label (update_remaining_time_label)
    """
    def countdown_timer_remaining_time(self, update_label_func):
        while self.remaining_time > 0:
            self.remaining_time -= 1
            self.root.after(0, update_label_func) # Update the GUI element for remaining time
            time.sleep(1)
        
        # Once the remaining time countdown finishes, reset the label
        self.root.after(0, self.update_remaining_time_label)
    
    """ Function used for count down the time and updates the UI label for repeat time (120 seconds)
        @param update_label_func function use to update the UI label (update_repeat_time_label)
    """
    def countdown_timer_repeat_time(self, update_label_func):
        while self.repeat_time > 0:
            self.repeat_time -= 1
            self.root.after(0, update_label_func) # Update the GUI element for repeat time
            time.sleep(1)
        
        # Once the repeat time countdown finishes, reset the label
        self.root.after(0, self.update_repeat_time_label)
    
    """ Function to cancel the previous repeat countdown thread """
    def cancel_previous_repeat_countdown_thread(self):
        if self.repeat_countdown_thread:
            self.repeat_time = 0 # Reset the repeat time to stop the previous thread
            self.repeat_countdown_thread.join() # Wait for the thread to finish

    """ Function to start the execution process based on the user input after the user press start button"""
    def start_execution(self):
        rospy.loginfo("Starting execution...")
        self.remaining_time = int(self.time_entry.get()) # get the set of time defined by user in UI

        if self.remaining_time <= 0:
            raise ValueError("Time must be positive!")
        
        simultaneous = self.simultaneous_var.get()
        sequential = self.sequential_var.get()
        repeat = self.repeat_var.get()
        rospy.loginfo(f"Execution will start in {self.remaining_time} seconds. Simultaneous: {simultaneous}, Sequential: {sequential}, Repeat: {repeat}")

        # Cancel previous repeat timer if it exists
        if self.repeat_timer:
            self.repeat_timer.cancel()
        
        # Reset repeat countdown
        self.repeat_time = 120 if repeat else 0
        self.update_repeat_time_label()

        # Start countdown timer thread
        threading.Thread(target=self.countdown_timer_remaining_time, args=(self.update_remaining_time_label,)).start()

        # Disable the start button so it can't accept new movement command from the User in UI until the robot finish their movement
        self.start_button.config(state=tk.DISABLED)

        # Start robot operation after delay from remaining time (calling start_robot_operation function)
        threading.Timer(self.remaining_time, self.start_robot_operation, args=(simultaneous, sequential, repeat)).start()

    """ Function to update the label showing the remaining time """
    def update_remaining_time_label(self):
        self.remaining_time_label.config(text=f"Remaining Time: {self.remaining_time} seconds")

    """ Function to update the label showing the time until the next repeat operation """
    def update_repeat_time_label(self):
        self.repeat_time_label.config(text=f"Repeat in: {self.repeat_time} seconds")

    """ Function to wait for both robots to finish their movements before re-enabling the start button."""
    def wait_for_both_robots(self):
        def check_robots():
            while self.robot1_moving or self.robot2_moving:
                time.sleep(1)
            self.root.after(0, self.enable_start_button)
        threading.Thread(target=check_robots).start()

    """ Function to enable the start button after both robots have finished their movements """
    def enable_start_button(self):
        self.start_button.config(state=tk.NORMAL)

if __name__ == "__main__":
    try:
        DualRobotSimulator()
    except rospy.ROSInterruptException:
        pass