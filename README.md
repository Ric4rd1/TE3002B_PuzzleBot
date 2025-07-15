# TE3002B_PuzzleBot 🤖
<p align="center">
  <img src="images/20250501_152858.png" alt="Left View" width="200"/>
  &nbsp;&nbsp;&nbsp;
  <img src="images/20250501_152910.png" alt="Front View" width="200"/>
  &nbsp;&nbsp;&nbsp;
  <img src="images/20250501_154338.png" alt="Right View" width="200"/>
</p>
This repository contains the packages and resources for the **MCR2 course** developed by **Manchester Robotics Ltd. (MCR2)**. The course is designed to provide students with an understanding of modern autonomous systems, combining theory and practical challenges.

## Mini Challenge 1 🎯⚡
#### 🎥[Video Link](https://drive.google.com/file/d/1wVKMqwvn5EAND5H07G1lQuohdTr_rPwj/view?usp=sharing)

Mini Challenge 1 consists of two main tasks:

1. **Square Path Challenge**  
   A ROS2 node is implemented to move the robot in a 2m x 2m square using an open-loop controller. The controller is auto-tuned based on user input (either speed or time) and is designed to be robust against noise, perturbations, and nonlinearities.

2. **Path Following Challenge**  
   A second node generates and follows custom paths defined by user parameters such as waypoints, velocities, or time. It calculates the required velocities or time, checks point reachability, and publishes a custom message based on `geometry_msgs/Pose` with additional fields for velocity or time.

The code for Mini Challenge 1 is located in the `mobile_robotics` package, and can be launched using the following launch files:
- **Square Path**: `move_square.launch.py`
- **Custom Path Following**: `path_challenge.launch.py`

## Mini Challenge 2 🎯⚡
#### 🎥[Video Link](https://drive.google.com/file/d/1sQrgKuxyGTh7qlWMnJJEpTJi8TXyb5x6/view?usp=sharing)

Mini Challenge 2 builds upon the tasks from Mini Challenge 1 by incorporating **closed-loop control** using feedback from encoders. This challenge focuses on executing the same square and custom paths, but now using closed-loop velocity control and position estimation. The robot now uses velocity measurements from the **inner control loop** available on the topics `/VelocityEncL` and `/VelocityEncR`.

A **new localization node** is introduced to estimate the robot’s position in real-time based on encoder feedback, significantly improving trajectory accuracy and robustness.

The implementation for Mini Challenge 2 is located in the `mobile_robotics_closed_loop` package and can be launched using the following files:
- **Closed-Loop Square Path**: `move_square.launch.py`
- **Closed-Loop Custom Path**: `move_path.launch.py`

## Mini Challenge 3 🎯⚡
#### 🎥[Video Link](https://drive.google.com/file/d/1X52T1fhW75T1G_6DVxZVCZMvHfgK_7GU/view?usp=sharing)

Mini Challenge 3 focuses on **closed-loop point-to-point navigation enhanced with visual traffic light detection**. The robot must navigate to predefined points using improved odometry and control logic, while also reacting to **color-based traffic lights** detected via its camera.

### Objectives
- Navigate to target points using closed-loop control.
- React to visual cues as traffic lights:
  - 🟢 **Green**: Continue moving.
  - 🟡 **Yellow**: Slow down.
  - 🔴 **Red**: Stop completely.

### Implementation Highlights
- **Color Detection Pipeline**:
  1. Apply Gaussian blur to the image.
  2. Convert to HSV and generate masks for red, yellow, and green.
  3. Use morphological operations (erode & dilate) to reduce noise.
  4. Perform blob detection to locate colored regions.

The code for Mini Challenge 3 is located in the `mobile_robotics_cv` package, and can be launched using the following launch files:
- **Point 2 Point reacting to colors**: `move_traffic_points.launch.py`

## Mini Challenge 4 🎯⚡
#### 🎥[Video Link](https://drive.google.com/file/d/1eKjnd2Q9a5Rks2hTrXPXeQjl1KMNvOAc/view?usp=sharing)

Mini Challenge 4 integrates **vision-based line following** with **traffic light detection**, requiring the robot to navigate a predefined track while obeying LED-based traffic signals.

### Objectives
- Follow a black line on a white surface using camera input.
- React to LED traffic lights using a custom-trained YOLOv8 model:
  - **Green**: Move forward.
  - **Yellow**: Slow down.
  - **Red**: Stop.

### Methodology
- Two **Regions of Interest (ROIs)** were defined:
  - One near the bottom of the frame to detect the line close to the robot.
  - A second in the middle of the frame to help calculate the heading error and detect corners.
- **Thresholding** was applied to isolate the black line.
- **Blob detection** was used to find the line and compute its centroid.
- The **angle between blobs** was used to infer corners and adjust heading.
- **Traffic lights** were detected using a **custom-trained YOLOv8 model**, allowing reliable identification of red, yellow, and green LED signals in real-time.

The code for Mini Challenge 4 is located in the `mobile_robotics_cv_line_tracker` package, and can be launched using the following launch files:
- **Line follower**: `move_line_traffic.launch.py`
- Ran directly on the **Puzzlebot** to minimize camera lag and improve real-time performance.

## Final challenge 🏁
#### 🎥[Video Link](https://drive.google.com/file/d/1CHFupGfjIkus1fNPz00tvlMVuwRiptf7/view?usp=sharing)

The final challenge implements all of the previous minichallenges, **vision-based line following** with **traffic light detection**, **traffic sign detection** and **intersection maneuvering**

### Objectives
- Follow a black line on a white surface using camera input.
- React to LED traffic lights using a custom-trained YOLOv8 model
	- **Green**: Move forward.
	- **Yellow**: Slow down.
	- **Red**: Stop.
- React to traffic Signs using custom trained YOLOv8 model
	- ** Right, Straight, Left ** directions ⬅️⬆️➡️: to move acordingly in intersections
	- ** Caution, Stop, Give way** directions⚠️🛑🚧: slow down or stop
- Detect intersections and act according to traffic lights and signs

### Methodology
- The same methodology for line following was used as the previous challenge
- Custom neural networks where used for object detection of traffic lights and signs
- Go 2 goal using odometry (encoders) aproach was used for moving on intersections

## More Videos 🎥
[LINK](https://drive.google.com/drive/folders/1_SqRldo4M0AQ8duqr3E6iTzbt0o8TAx1?usp=sharing)


## Puzzlebot Hardware ⚙️

The Puzzlebot system is powered by the following hardware components:

- **Jetson Nano 2GB**: A small yet powerful AI computer for running ROS2 and ML applications.
- **Hackerboard (ESP32, Motor Driver)**: Used to control the robot's motors and sensors.
- **2 DC Motoreductors**: [Pololu 4824 DC Motor Driver](https://www.pololu.com/product/4824) to drive the robot's wheels.

## Software 🖥️

We are using the following software for the course:

- **ROS2 Humble**: A powerful robotics framework for controlling autonomous systems.
- **Ubuntu 22.04**: The operating system running on the Jetson Nano to host ROS2.

## Links for Firmware

Here are the links to download the firmware for the **Hackerboard** and **Jetson Nano**:

- **Hackerboard Firmware**: [Download Link](https://tecmx-my.sharepoint.com/personal/mario_mtz_tec_mx/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fmario%5Fmtz%5Ftec%5Fmx%2FDocuments%2Fpuzzlebot%5Ffirmware&ga=1)  
- **Jetson Nano Firmware**: [Download Link](https://manchesterrobotics-my.sharepoint.com/personal/mario_mtz_manchester-robotics_com/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fmario%5Fmtz%5Fmanchester%2Drobotics%5Fcom%2FDocuments%2FManchester%20Robotics%2FTeaching%20and%20learning%2FCourses%2FCADI%20ROS2%2FCADI%20%2D%20Invierno%2FActivities%2Fjetson%5F2gb%5Fubuntu20%2Ezip&parent=%2Fpersonal%2Fmario%5Fmtz%5Fmanchester%2Drobotics%5Fcom%2FDocuments%2FManchester%20Robotics%2FTeaching%20and%20learning%2FCourses%2FCADI%20ROS2%2FCADI%20%2D%20Invierno%2FActivities&ga=1)
