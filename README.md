# Load Cell with ROS2

A simple ROS2 package that reads weight data from a **Load Cell using HX711** and publishes it to a ROS2 topic through serial communication with a microcontroller (Arduino/ESP).

---

## System Architecture

Load Cell → HX711 → Arduino / ESP → Serial → ROS2 Node → /load_cell_data

The ROS2 node reads the serial data and publishes the weight value as a ROS2 message.

---

## Features

- Load cell integration with ROS2
- HX711 amplifier support
- Serial communication with microcontroller
- Real-time weight publishing to `/load_cell_data`
- Useful for robotics and AMR applications

---

## Repository Structure

load_cell_pkg/
│
├── config/
├── launch/
├── load_cell_pkg/        # ROS2 Python node
├── resource/
├── test/
│
├── load_cell.ino         # Arduino firmware
├── package.xml
├── setup.py
└── setup.cfg

---

## Hardware Required

- Load Cell
- HX711 Amplifier
- Arduino / ESP microcontroller
- Computer running ROS2

---

## Installation

Clone the repository inside your ROS2 workspace:

cd ~/ros2_ws/src
git clone https://github.com/ghanshyamjadhav755-svg/load-cell-with-ros2.git

Build the package:

cd ~/ros2_ws
colcon build
source install/setup.bash

---

## Run the Node

ros2 run load_cell_pkg loadcell_publisher

Check the published data:

ros2 topic echo /load_cell_data

---

## Applications

- Robot payload detection
- Warehouse automation
- Material handling systems
- AMR weight monitoring
