# Robotics Assignment 4: Lab Execution Guide

This guide details the exact procedure for running your autonomous pick-and-place demonstration on the physical TurtleBot3 hardware in the robotics lab.

## 0. Preparation
1. Identify a target object (e.g., a bottle or cup) and place it on the floor.
2. Ensure the TurtleBot3 and the OpenManipulator-X arm are both powered on.
3. Establish a connection to the Nvidia Jetson (e.g., via SSH from your Remote PC or using a direct monitor/keyboard).
4. Navigate to your cloned workspace directory:
   ```bash
   cd ~/Projects/Robotics_Assignment_4
   ```

## 1. Start Hardware Drivers (Terminal 1)
First, initialize the hardware interfaces for the base, camera, and robotic arm. 
*(Use the exact bring-up command provided by your lab instructor. Usually, it resembles the following):*
```bash
# Example bring-up commands for Humble:
ros2 launch turtlebot3_bringup robot.launch.py
# In a separate terminal if required for the arm:
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

## 2. Start the YOLOv11 Vision Node (Terminal 2)
In a new terminal window (don't forget to source your ROS 2 environment), run the publisher script. This script will tap into the camera stream, load the `yolo11s.pt` model onto the Jetson's GPU, and begin broadcasting detection data.
```bash
python3 yolo_publisher_sample_code.py
```
> **Note:** The very first time you execute this on the lab's Jetson, it may take 10-20 seconds to download the `yolo11s.pt` weights file and initialize the PyTorch CUDA tensors. 

## 3. Start the Autonomous Controller (Terminal 3)
In a third terminal window, launch your main state machine script. This script acts as both your manual teleoperation node and your autonomous brain.
```bash
python3 sample_code.py
```

## 4. Operation & Demonstration
Once the `sample_code.py` script is running, make sure your terminal is selected to capture keyboard strokes. 

### Manual Overrides
Use these to position the robot before starting your demonstration:
- **`w` / `a` / `s` / `d`** : Drive the base.
- **`g` / `h`** : Open and close the gripper.
- **`9`** : Return arm to Home pose (safe driving mode).
- **`0`** : Extend the arm forward.
- **`q`** : Safely issue a zero-velocity twist and exit the script.

### Autonomous Demonstration
To fulfill the assignment criteria and show the TA, press one of the following number keys:
- **`1` (Auto Visual Servo):** Automatically steers the robot left/right to lock onto and center the target object (e.g., bottle) in the camera frame.
- **`2` (Auto Pick):** Visual servos toward the object, drives forward until the bounding box exceeds the threshold, and automatically executes the arm sequence to grab it and return to the home pose. 
- **`3` (Auto Pick & Place):** Executes the full Auto Pick sequence, then automatically turns around 180 degrees, drives back the exact distance/time it took to approach the object, and executes a placement sequence to set it back down.
- **`c`** : Immediately cancel the autonomous mode and halt.

## Troubleshooting
- **Robot is not detecting the object:** Make sure the object matches the classes in your whitelist (`bottle`, `bear`, `mouse`, `teddy bear`, `donut`, `cup`, `cell phone`) and the room is well-lit.
- **Arm is not moving:** Verify that the Action Servers (`/arm_controller/follow_joint_trajectory` and `/gripper_controller/gripper_cmd`) successfully started without crashing in Terminal 1.
- **Robot spins erratically:** Ensure the camera is mounted perfectly straight. If it's tilted, the `x_center` calculation will constantly try to correct an impossible offset.
