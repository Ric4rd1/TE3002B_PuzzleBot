# TE3002B_PuzzleBot
This repository contains the packages and resources for the **MCR2 course** developed by **Manchester Robotics Ltd. (MCR2)**. The course is designed to provide students with an understanding of modern autonomous systems, combining theory and practical challenges.

## Overview

The course is structured into **ten sessions**, each aimed at teaching students various aspects of robotics, ranging from basic navigation techniques to more advanced topics such as image recognition and machine learning implementation. Throughout the course, students will work with the **Puzzlebot** platform, leveraging **ROS2** (Humble) and **Ubuntu 22.04** to implement intelligent algorithms in robotics.

Each session includes hands-on challenges that simulate real-world problems faced during the implementation of advanced algorithms in autonomous robotics.

## Mini Challenge 1

Mini Challenge 1 consists of two main tasks:

1. **Square Path Challenge**  
   A ROS2 node is implemented to move the robot in a 2m x 2m square using an open-loop controller. The controller is auto-tuned based on user input (either speed or time) and is designed to be robust against noise, perturbations, and nonlinearities.

2. **Path Following Challenge**  
   A second node generates and follows custom paths defined by user parameters such as waypoints, velocities, or time. It calculates the required velocities or time, checks point reachability, and publishes a custom message based on `geometry_msgs/Pose` with additional fields for velocity or time.

The code for Mini Challenge 1 is located in the `mobile_robotics` package, and can be launched using the following launch files:
- **Square Path**: `move_square.launch.py`
- **Custom Path Following**: `path_challenge.launch.py`

## Puzzlebot Hardware

The Puzzlebot system is powered by the following hardware components:

- **Jetson Nano 2GB**: A small yet powerful AI computer for running ROS2 and ML applications.
- **Hackerboard (ESP32, Motor Driver)**: Used to control the robot's motors and sensors.
- **2 DC Motoreductors**: [Pololu 4824 DC Motor Driver](https://www.pololu.com/product/4824) to drive the robot's wheels.

## Software

We are using the following software for the course:

- **ROS2 Humble**: A powerful robotics framework for controlling autonomous systems.
- **Ubuntu 22.04**: The operating system running on the Jetson Nano to host ROS2.

## Links for Firmware

Here are the links to download the firmware for the **Hackerboard** and **Jetson Nano**:

- **Hackerboard Firmware**: [Download Link](https://tecmx-my.sharepoint.com/personal/mario_mtz_tec_mx/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fmario%5Fmtz%5Ftec%5Fmx%2FDocuments%2Fpuzzlebot%5Ffirmware&ga=1)  
- **Jetson Nano Firmware**: [Download Link](https://manchesterrobotics-my.sharepoint.com/personal/mario_mtz_manchester-robotics_com/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fmario%5Fmtz%5Fmanchester%2Drobotics%5Fcom%2FDocuments%2FManchester%20Robotics%2FTeaching%20and%20learning%2FCourses%2FCADI%20ROS2%2FCADI%20%2D%20Invierno%2FActivities%2Fjetson%5F2gb%5Fubuntu20%2Ezip&parent=%2Fpersonal%2Fmario%5Fmtz%5Fmanchester%2Drobotics%5Fcom%2FDocuments%2FManchester%20Robotics%2FTeaching%20and%20learning%2FCourses%2FCADI%20ROS2%2FCADI%20%2D%20Invierno%2FActivities&ga=1)
