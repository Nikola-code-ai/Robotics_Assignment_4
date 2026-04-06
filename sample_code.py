# Sample Code for Robotics_Assignment_3
# Copyright: 2026 CS 4379K / CS 5342 Introduction to Autonomous Robotics, Robotics and Autonomous Systems

# Teleoperation Code for Turtlebot3 for Assignment 3.

# This code will give you an introduction to coding in ROS2. This code will run as-is, but you are required to modify it for the assignment requirement and submit the source code on Canvas.

# Refer to the Lab PowerPoint materials and Appendix of Assignment 3 to learn more about coding on ROS2 and the hardware architecture of Turtlebot3.
# You can either run this code on the simulator, Turtlebot3 Nvidia Jetson, or on a Remote-PC docker image.
# You would need a basic understanding of Python Data Structure and Object Oriented Programming to understand this code.

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# ROS2 Programming
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
    # Advice: You will need geometry_msgs for the Twist message to control the base
from geometry_msgs.msg import Twist
    # Advice: You will need sensor_msgs for JointState to read the arm angles
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json

# Terminal related
import sys
import tty
import termios
import select

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

# We are giving you complete constants for the gripper to save you the trouble. 
GRIPPER_KEY_BINDINGS = {
    'g': 0.01,  # Open
    'h': -0.01  # Close
}

POSES = {
    '9': [0.0, 0.0, 0.0, 0.0],        # Home pose
    '0': [0.0, 0.4, 0.2, -0.6],        # Extend Forward
    '8': [0.0, -0.5, -0.5, 0.0],       # Wave pose
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class SimpleDemoController(Node):
    def __init__(self):

        # Initialize and Define Node name
        super().__init__('simple_demo_controller')

        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # State Variables
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        
        self.current_j1 = 0.0
        self.current_j2 = 0.0
        self.current_j3 = 0.0
        self.current_j4 = 0.0

        # Action Clients
        self.arm_action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # YOLO Subscriber for Autonomous Mode
        self.yolo_sub = self.create_subscription(String, '/yolo/detections_json', self.yolo_callback, 10)
        self.latest_bottle = None
        self.auto_mode = 0  # 0: off, 1: servo, 2: pick, 3: pick & place
        self.auto_state = 'DONE'
        self.state_timer = 0
        self.approach_time = 0

        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = self.create_timer(0.1, self.run_loop)
        self.print_instructions()

    def joint_state_callback(self, msg):
        
        # Extract the current joint angles from the robot
        if 'joint1' in msg.name:
            idx = msg.name.index('joint1')
            self.current_j1 = msg.position[idx]

        if 'joint2' in msg.name:
            idx = msg.name.index('joint2')
            self.current_j2 = msg.position[idx]

        if 'joint3' in msg.name:
            idx = msg.name.index('joint3')
            self.current_j3 = msg.position[idx]

        if 'joint4' in msg.name:
            idx = msg.name.index('joint4')
            self.current_j4 = msg.position[idx]

    def yolo_callback(self, msg):
        try:
            data = json.loads(msg.data)
            bottle_det = None
            for det in data.get("detections", []):
                cls = det.get('class_name', '')
                if cls in ['bottle', 'bear', 'mouse', 'teddy bear', 'donut', 'cup', 'cell phone']:
                    if bottle_det is None or det.get('confidence', 0) > bottle_det.get('confidence', 0):
                        bottle_det = det
            self.latest_bottle = bottle_det
        except Exception as e:
            pass

    def execute_auto_logic(self):
        if self.state_timer > 0:
            self.state_timer -= 1

        match self.auto_state:
            case 'SERVO':
                if self.latest_bottle:
                    cx = self.latest_bottle['bbox']['cx']
                    err = 640.0 - cx
                    self.target_angular_vel = err * 0.0015
                    self.target_linear_vel = 0.0
                else:
                    self.target_angular_vel = 0.0
                    self.target_linear_vel = 0.0
                    
            case 'APPROACH':
                if self.latest_bottle:
                    cx = self.latest_bottle['bbox']['cx']
                    cy = self.latest_bottle['bbox']['cy']
                    h = self.latest_bottle['bbox']['h']
                    w = self.latest_bottle['bbox']['w']
                    err = 640.0 - cx
                    self.target_angular_vel = err * 0.0015
                    
                    # Check if close enough
                    if cy > 450 or w > 300 or h > 300: 
                        self.target_linear_vel = 0.0
                        self.target_angular_vel = 0.0
                        self.auto_state = 'PICK_STEP1'
                        self.state_timer = 0
                    else:
                        if abs(err) < 80:
                            self.target_linear_vel = 0.03
                        else:
                            self.target_linear_vel = 0.0
                    self.approach_time += 1
                else:
                    self.target_linear_vel = 0.0
                    self.target_angular_vel = 0.0
                    
            case 'PICK_STEP1':
                if self.state_timer <= 0:
                    self.send_arm_goal(POSES['9'], 2.0)
                    self.auto_state = 'PICK_STEP2'
                    self.state_timer = 25
            case 'PICK_STEP2':
                if self.state_timer <= 0:
                    self.send_gripper_goal(0.01)
                    self.auto_state = 'PICK_STEP3'
                    self.state_timer = 15
            case 'PICK_STEP3':
                if self.state_timer <= 0:
                    self.send_arm_goal(POSES['0'], 2.0)
                    self.auto_state = 'PICK_STEP4'
                    self.state_timer = 25
            case 'PICK_STEP4':
                if self.state_timer <= 0:
                    self.send_gripper_goal(-0.01)
                    self.auto_state = 'PICK_STEP5'
                    self.state_timer = 15
            case 'PICK_STEP5':
                if self.state_timer <= 0:
                    self.send_arm_goal(POSES['9'], 2.0)
                    if self.auto_mode == 3:
                        self.auto_state = 'RETURN_TURN'
                        self.state_timer = 25
                    else:
                        self.auto_state = 'DONE'
                        self.auto_mode = 0
            
            case 'RETURN_TURN':
                if self.state_timer <= 0:
                    self.target_linear_vel = 0.0
                    self.target_angular_vel = 0.3
                    self.state_timer = 104
                    self.auto_state = 'RETURN_DRIVE'
            case 'RETURN_DRIVE':
                if self.state_timer <= 0:
                    self.target_angular_vel = 0.0
                    self.target_linear_vel = 0.03
                    self.state_timer = self.approach_time
                    self.auto_state = 'PLACE_STEP1'
            case 'PLACE_STEP1':
                if self.state_timer <= 0:
                    self.target_linear_vel = 0.0
                    self.target_angular_vel = 0.0
                    self.send_arm_goal(POSES['0'], 2.0)
                    self.state_timer = 25
                    self.auto_state = 'PLACE_STEP2'
            case 'PLACE_STEP2':
                if self.state_timer <= 0:
                    self.send_gripper_goal(0.01)
                    self.state_timer = 15
                    self.auto_state = 'PLACE_STEP3'
            case 'PLACE_STEP3':
                if self.state_timer <= 0:
                    self.send_arm_goal(POSES['9'], 2.0)
                    self.auto_state = 'DONE'
                    self.auto_mode = 0
            
            case 'DONE':
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0

    def run_loop(self):
        key = get_key(self.settings)

        match key:
            case '1':
                self.auto_mode = 1
                self.auto_state = 'SERVO'
                self.get_logger().info("Starting Auto Visual Servo")
            case '2':
                self.auto_mode = 2
                self.auto_state = 'APPROACH'
                self.approach_time = 0
                self.get_logger().info("Starting Auto Pick")
            case '3':
                self.auto_mode = 3
                self.auto_state = 'APPROACH'
                self.approach_time = 0
                self.get_logger().info("Starting Auto Pick & Place")
            case 'c':
                self.auto_mode = 0
                self.auto_state = 'DONE'
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0
                self.get_logger().info("Cancelled Auto Mode")
            case 's' if self.auto_mode != 0:
                self.auto_mode = 0
                self.auto_state = 'DONE'
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0
                self.get_logger().info("Cancelled Auto Mode")

        if self.auto_mode != 0:
            self.execute_auto_logic()
            twist = Twist()
            twist.linear.x = self.target_linear_vel
            twist.angular.z = self.target_angular_vel
            self.cmd_vel_pub.publish(twist)
            self.print_status()
            return

        if not key:
            return

        # Base Control Logic
        match key:
            case 'w':
                self.target_linear_vel += LIN_VEL_STEP_SIZE
            case 'x':
                self.target_linear_vel -= LIN_VEL_STEP_SIZE
            case 'a':
                self.target_angular_vel += ANG_VEL_STEP_SIZE
            case 'd':
                self.target_angular_vel -= ANG_VEL_STEP_SIZE
            case 's':
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0

            # Arm & Gripper Logic
            case _ if key in GRIPPER_KEY_BINDINGS:
                self.send_gripper_goal(GRIPPER_KEY_BINDINGS[key])
            
            case _ if key in POSES:
                self.send_arm_goal(POSES[key], 2.0)
                
            case 'q' | 'Q':
                # Stop the robot before quitting
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.destroy_node()
                rclpy.shutdown()
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                sys.stdout.write('\nExiting...\n')
                sys.exit(0)

        # Publish Twist Message 
        twist = Twist()
        twist.linear.x = self.target_linear_vel
        twist.angular.z = self.target_angular_vel
        self.cmd_vel_pub.publish(twist)

        # Print the current status to the terminal
        self.print_status()

    def send_arm_goal(self, positions, duration_sec):
        if not self.arm_action_client.server_is_ready():
            self.get_logger().info("Arm action server not available")
            return
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint(
            positions=positions, 
            time_from_start=Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        )
        goal.trajectory.points.append(point)
        self.arm_action_client.send_goal_async(goal)

    def send_gripper_goal(self, position):
        if not self.gripper_action_client.server_is_ready():
            self.get_logger().info("Gripper action server not available")
            return
            
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 1.0
        self.gripper_action_client.send_goal_async(goal)

    def print_status(self):
        # Clear the previous line and print the updated status
        sys.stdout.write('\r' + ' ' * 80 + '\r') # Clear line
        status_string = (f"Auto Mode: {self.auto_mode} | Auto State: {self.auto_state}\n"
                         f"Present Linear Velocity: {self.target_linear_vel:.3f}, Angular Velocity: {self.target_angular_vel:.3f}\n"
                         f"Present Arm Joint Angle J1: {self.current_j1:.3f} J2: {self.current_j2:.3f} J3: {self.current_j3:.3f} J4: {self.current_j4:.3f}\n"
                         f"---------------------------\n")

        # Moves cursor up 4 lines so the next print overwrites it cleanly
        sys.stdout.write(status_string + "\033[4A")
        sys.stdout.flush()

    def print_instructions(self):
        print("""
---------------------------
 Teleoperation Control of TurtleBot3 + OpenManipulatorX
---------------------------
 Base
 w : increase linear velocity
 x : decrease linear velocity
 a : increase angular velocity
 d : decrease angular velocity
 s : base stop

 Gripper
 g : gripper open
 h : gripper close

 Arm Preset
 0 : Extend Forward
 9 : Home pose
 8 : Wave pose

 Auto Modes
 1 : Auto Visual Servo
 2 : Auto Pick
 3 : Auto Pick & Place
 c : Cancel Auto Mode

 q to quit
---------------------------
        """)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDemoController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
