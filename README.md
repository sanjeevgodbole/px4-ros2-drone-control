# Autonomous Drone Control with PX4 and ROS 2

This project demonstrates autonomous control of a PX4-piloted drone in a Gazebo simulation environment using ROS 2. The core of the project is a Python-based ROS 2 node that implements a closed-loop control system to command the drone through a predefined waypoint mission.

It utilizes PX4's **Offboard mode** to send high-level trajectory setpoints, enabling complex flight paths without manual intervention. This repository contains the source code for the ROS 2 control node and instructions for setting up the complete Software-in-the-Loop (SITL) simulation environment.

![Demo GIF](https://via.placeholder.com/600x400.png?text=Add+a+GIF+of+your+drone+flying+here)
*(Replace the image above with a screen recording or GIF of your simulation)*

---

## 📋 Key Features

- **Software-in-the-Loop (SITL) Simulation**: Utilizes the complete PX4, Gazebo, and ROS 2 toolchain for safe, hardware-free development.
- **Closed-Loop Waypoint Navigation**: Implements a robust state machine that navigates a multi-point mission by using the drone's real-time position feedback.
- **Offboard Control**: Leverages PX4's Offboard mode to accept high-level trajectory commands from the ROS 2 node.
- **GPS Integration**: The control node subscribes to the drone's simulated GPS sensor to read and store its global position.

---

## 🛠️ Technologies Used

- **ROS 2 Jazzy Jalisco**: The core robotics middleware.
- **PX4 Autopilot**: The flight controller firmware running in SITL mode.
- **Gazebo Harmonic**: The 3D robotics simulator.
- **QGroundControl**: The ground control station for vehicle monitoring.
- **Python 3**: For the offboard control node.
- **Micro XRCE-DDS Agent**: The middleware bridge between ROS 2 and PX4.

---

## 🚀 Setup and Installation

This project was developed on **Ubuntu 24.04** with **ROS 2 Jazzy**.

### 1. Install the PX4 Toolchain
This step installs the PX4 source code, Gazebo simulator, and all required system dependencies.

```bash
# Clone the PX4-Autopilot repository
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot/Tools/setup
# Run the official setup script
bash ubuntu.sh
```

### 2. Install Ground Control Station (QGroundControl)
QGroundControl is used for mission planning and monitoring the vehicle's status. It is required to pass the PX4 pre-flight health checks before arming.

- Download the QGroundControl AppImage from the [official website](http://qgroundcontrol.com/downloads/).
- Make the AppImage executable:
```bash
chmod +x ./QGroundControl.AppImage
```

### 3. Install px4_msgs from Source
The custom ROS 2 messages for PX4 must be built from source in a separate workspace.

```bash
# Create and navigate to a new workspace
mkdir -p ~/px4_msgs_ws/src
cd ~/px4_msgs_ws/src

# Clone the repository
git clone https://github.com/PX4/px4_msgs.git

# Build the workspace
cd ..
colcon build
```

### 4. Install the Micro XRCE-DDS Agent
This agent is the communication bridge that connects the PX4 flight controller to the ROS 2 network.

```bash
# Navigate to your home directory and clone the repository
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

# Build and install the agent
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install

# Refresh the system's library cache
sudo ldconfig
```

### 5. Clone This Project
Finally, clone this project's repository into your main ROS 2 workspace.

```bash
# Example workspace setup
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/sanjeevgodbole/px4-ros2-drone-control.git
```

---

## ▶️ How to Run

To launch the full simulation and run the mission, you will need **3 terminals** and the **QGroundControl** application.

### 1. Launch the PX4/Gazebo Simulation

```bash
# Set the ROS_DOMAIN_ID and launch the simulation
export ROS_DOMAIN_ID=0
cd /path/to/your/PX4-Autopilot
make px4_sitl gz_x500
```

> **Why export ROS_DOMAIN_ID=0?** ROS 2 uses this variable to create isolated communication networks. By setting this ID, we ensure that the PX4 simulator, the Agent, and our ROS 2 nodes are all on the same network, allowing them to communicate.

Wait for the Gazebo window to appear and for the PX4 terminal to finish its startup sequence.

### 2. Start QGroundControl

Run the QGroundControl AppImage you downloaded. It should automatically detect and connect to the running simulation. Wait for the drone's attitude display (the artificial horizon) to become active.

### 3. Launch the Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 4. Build and Run the Control Node

```bash
# In a NEW terminal, first source the px4_msgs workspace
source ~/px4_msgs_ws/install/setup.bash

# Navigate to your project workspace (e.g., ~/ros2_ws)
cd /path/to/your/ros2_ws

# Build the offboard_control package
colcon build --packages-select offboard_control

# Source the overlay workspace and run the node
source install/setup.bash
ros2 run offboard_control offboard_control_node
```

The drone will now arm, take off, fly the mission, and land in the Gazebo simulation.

---

## 📝 Code Overview

The main logic is contained within the `offboard_control/offboard_control_node.py` file. The node is structured as a class that implements a state machine to guide the drone through the mission.

- **Publishers**: The node uses ROS 2 publishers to send commands to the PX4 flight controller, such as trajectory setpoints and high-level commands like arm or land.
- **Subscribers**: To enable closed-loop control, the node subscribes to feedback from the drone, using `vehicle_odometry` for local position and `sensor_gps` for global position.
- **State Machine**: The mission is managed by a state machine with states such as `TAKEOFF`, `FLYING_WAYPOINTS`, and `LANDING`. The node transitions between states based on the drone's actual position relative to its target.
- **Waypoint List**: The entire flight path is defined in an easy-to-edit Python list of `[x, y, z]` coordinates at the top of the file, making the mission highly configurable.

---

## 📄 License

This project is licensed under the MIT License. See the LICENSE file for details.