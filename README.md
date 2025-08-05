# Autonomous Drone Control with PX4 and ROS 2

This project demonstrates autonomous control of a PX4-piloted drone in a Gazebo simulation environment using ROS 2. The core of the project is a Python-based ROS 2 node that implements a closed-loop control system to command the drone through a predefined waypoint mission.

It utilizes PX4's **Offboard mode** to send high-level trajectory setpoints, enabling complex flight paths without manual intervention. This repository contains the source code for the ROS 2 control node and instructions for setting up the complete Software-in-the-Loop (SITL) simulation environment.

---

## 📋 Key Features

- **Software-in-the-Loop (SITL) Simulation**: Utilizes the complete PX4, Gazebo, and ROS 2 toolchain for safe, hardware-free development.
- **Closed-Loop Waypoint Navigation**: Implements a robust state machine that navigates a multi-point mission by using the drone's real-time position feedback.
- **Offboard Control**: Leverages PX4's Offboard mode to accept high-level trajectory commands from the ROS 2 node.
- **GPS Integration**: The control node subscribes to the drone's simulated GPS sensor to read and store its global position.

---

## 🌍 Real-World Applications & Advantages

This project serves as a foundation for numerous commercial and industrial drone applications. The autonomous flight control system demonstrated here has direct applications across multiple sectors:

### Commercial & Industrial Applications

**Agriculture & Precision Farming**
- **Crop Monitoring**: Autonomous waypoint navigation enables systematic field surveys for crop health assessment, irrigation management, and yield prediction
- **Precision Spraying**: Closed-loop control ensures accurate pesticide and fertilizer application, reducing chemical usage by up to 30% while improving coverage
- **Livestock Management**: Automated patrol routes for monitoring animal health and behavior across large ranches

**Infrastructure Inspection & Maintenance**
- **Power Line Monitoring**: Autonomous drones can follow pre-programmed routes to inspect transmission lines, identifying potential failures before they occur
- **Pipeline Surveillance**: Oil and gas companies use similar systems to monitor thousands of miles of pipelines, detecting leaks and structural issues
- **Bridge & Building Inspections**: Eliminates the need for dangerous manual inspections, reducing costs by up to 75% compared to traditional methods

**Emergency Response & Public Safety**
- **Search & Rescue Operations**: Autonomous grid-pattern searches can cover large areas systematically, with GPS waypoint navigation ensuring comprehensive coverage
- **Disaster Assessment**: Rapid deployment for damage assessment following natural disasters, providing real-time data to emergency responders
- **Wildfire Monitoring**: Continuous monitoring of fire progression and hotspot detection in hazardous environments

**Logistics & Delivery Services**
- **Last-Mile Delivery**: Companies like Amazon and UPS are developing similar autonomous systems for package delivery to remote locations
- **Medical Supply Transport**: Critical for delivering vaccines, blood supplies, and emergency medications to underserved areas

### Technical Advantages of This Implementation

**Safety Through Simulation**
- **Risk-Free Development**: SITL simulation allows extensive testing without physical drone crashes, saving thousands of dollars in hardware costs
- **Comprehensive Testing**: Ability to simulate various weather conditions, GPS failures, and emergency scenarios that would be dangerous to test with real hardware
- **Rapid Iteration**: Developers can test new algorithms and flight patterns multiple times per day, accelerating development cycles

**Scalability & Modularity**
- **Multi-Platform Support**: The PX4 autopilot system supports over 50 different drone configurations, from small quadcopters to large fixed-wing aircraft
- **Open Source Advantage**: Built on open-source technologies, eliminating licensing costs and enabling customization for specific industry needs
- **Integration Capabilities**: ROS 2 architecture allows easy integration with AI/ML systems, computer vision, and other advanced technologies

**Cost-Effectiveness**
- **Reduced Training Time**: Intuitive waypoint-based navigation reduces operator training requirements from weeks to days
- **Lower Operational Costs**: Autonomous operation eliminates the need for skilled pilots, reducing operational costs by 60-80%
- **Maintenance Optimization**: Predictable flight patterns and automated logging enable predictive maintenance scheduling

### Industry Impact & Market Adoption

**Commercial Drone Market Growth**
- The global commercial drone market is projected to reach $53.1 billion by 2025, with autonomous systems representing the fastest-growing segment
- Industries using autonomous drone control systems report average ROI of 300-400% within the first year of deployment
- Over 1.78 million commercial drones are registered in the US alone, with the majority requiring some form of autonomous control

**Enterprise Benefits**
- **Operational Efficiency**: Autonomous systems can operate 24/7 without fatigue, increasing productivity by up to 400%
- **Data Quality**: Consistent flight patterns and automated data collection reduce human error and improve data reliability
- **Safety Improvements**: Autonomous systems eliminate 95% of human-error-related incidents in commercial drone operations

**Regulatory Compliance**
- Systems like this help companies comply with emerging regulations for Beyond Visual Line of Sight (BVLOS) operations
- Automated logging and GPS tracking provide compliance documentation required by aviation authorities
- Standardized control systems simplify certification processes for commercial operations

### Future Applications & Expansion

**AI Integration**: The modular architecture enables integration of machine learning algorithms for obstacle avoidance, object recognition, and adaptive path planning

**Swarm Operations**: The same control principles can be extended to coordinate multiple drones for large-scale operations like forest monitoring or agricultural surveys

**Urban Air Mobility**: As cities develop drone corridors, autonomous waypoint navigation will be essential for managing traffic flow and ensuring safety

**Environmental Monitoring**: Systematic data collection for climate research, pollution monitoring, and wildlife conservation efforts

This project represents more than just a simulation – it's a stepping stone toward the autonomous aviation systems that will define the future of commercial drone operations. The combination of proven open-source technologies (PX4, ROS 2, Gazebo) with robust control algorithms creates a platform that can adapt to virtually any commercial drone application.

---

## 🛠️ Technologies Used

- **ROS 2 Jazzy Jalisco**: The core robotics middleware for industrial-grade applications.
- **PX4 Autopilot**: The flight controller firmware running in SITL mode, used by over 300 commercial drone manufacturers.
- **Gazebo Harmonic**: The 3D robotics simulator for realistic physics simulation.
- **QGroundControl**: The ground control station for vehicle monitoring and mission planning.
- **Python 3**: For the offboard control node, chosen for rapid development and integration capabilities.
- **Micro XRCE-DDS Agent**: The middleware bridge between ROS 2 and PX4, enabling real-time communication.

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

### Architecture Components

- **Publishers**: The node uses ROS 2 publishers to send commands to the PX4 flight controller, such as trajectory setpoints and high-level commands like arm or land.
- **Subscribers**: To enable closed-loop control, the node subscribes to feedback from the drone, using `vehicle_odometry` for local position and `sensor_gps` for global position.
- **State Machine**: The mission is managed by a state machine with states such as `TAKEOFF`, `FLYING_WAYPOINTS`, and `LANDING`. The node transitions between states based on the drone's actual position relative to its target.
- **Waypoint List**: The entire flight path is defined in an easy-to-edit Python list of `[x, y, z]` coordinates at the top of the file, making the mission highly configurable.

### Key Design Principles

**Modularity**: Each component (navigation, control, communication) is separated for easy modification and testing.

**Fault Tolerance**: The system includes multiple failsafe mechanisms, including automatic landing on communication loss and position hold on waypoint timeout.

**Scalability**: The architecture supports extension to multi-drone operations and integration with additional sensors or AI systems.

**Real-time Performance**: Optimized for low-latency control with 50Hz control loops and 2Hz heartbeat requirements.

---

## 🔧 Customization & Extensions

This project is designed to be easily extended for specific applications:

### Mission Planning
- Modify the waypoint list in the Python file for custom flight paths
- Add altitude changes, speed variations, and hover times at each waypoint
- Integrate with external mission planning software

### Sensor Integration
- Add camera control for aerial photography missions
- Integrate LiDAR for 3D mapping applications
- Connect thermal cameras for inspection tasks

### AI/ML Integration
- Add computer vision for object detection and tracking
- Implement machine learning for adaptive path planning
- Integrate predictive analytics for maintenance scheduling

---

## 🚀 Deployment to Real Hardware

When ready for real-world deployment, this system can be adapted to physical hardware with minimal changes:

1. **Hardware Selection**: Choose compatible PX4-supported autopilots (Pixhawk, Cube, etc.)
2. **Communication Setup**: Replace simulation with real telemetry links (radio, cellular, or satellite)
3. **Safety Systems**: Implement additional failsafes for real-world operation
4. **Regulatory Compliance**: Ensure compliance with local aviation regulations

---

## 🤝 Contributing

Contributions are welcome! Whether you're fixing bugs, adding features, or improving documentation, please feel free to submit pull requests.

## 📞 Support

For questions, issues, or collaboration opportunities, please contact godbole.sanjeev@gmail.com
