import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from rclpy.clock import Clock
import time

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self):
        super().__init__('offboard_control_house_mission')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Define the flight path as a list of waypoints [x, y, z]
        self.waypoints = [
            [0.0, 5.0, -5.0],    # 1. Fly to front-left corner
            [5.0, 5.0, -5.0],    # 2. Fly to front-right corner
            [5.0, 0.0, -5.0],    # 3. Fly to back-right corner
            [0.0, 0.0, -5.0],    # 4. Fly to back-left corner (completes square)
            [2.5, -2.5, -7.0],   # 5. Fly up to the roof peak
            [5.0, 0.0, -5.0]     # 6. Fly back down to the back-right corner
        ]
        
        # Initialize state machine variables
        self.state = "IDLE"
        self.waypoint_index = 0
        self.mission_timer = 0
        self.last_state_change_time = 0

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("House mission node has been started.")

    def publish_vehicle_command(self, command, **params):
        # ... (this function remains the same)
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self): # ... (same as before)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self): # ... (same as before)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
    
    def engage_offboard_mode(self): # ... (same as before)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to Offboard mode")

    def land(self): # ... (same as before)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def timer_callback(self):
        # Publish offboard control mode continuously
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(offboard_msg)

        trajectory_msg = TrajectorySetpoint()
        
        if self.state == "IDLE":
            if self.mission_timer == 10: # After 1 second
                self.engage_offboard_mode()
                self.arm()
                self.state = "TAKEOFF"
                self.last_state_change_time = self.mission_timer

        elif self.state == "TAKEOFF":
            trajectory_msg.position = [0.0, 0.0, -5.0]
            if self.mission_timer - self.last_state_change_time >= 50: # After 5 seconds
                self.get_logger().info("Takeoff complete. Starting waypoint mission.")
                self.state = "FLYING_WAYPOINTS"
                self.last_state_change_time = self.mission_timer
        
        elif self.state == "FLYING_WAYPOINTS":
            # Check if we have completed all waypoints
            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().info("All waypoints reached. Landing.")
                self.land()
                self.state = "LANDING"
                self.last_state_change_time = self.mission_timer
            else:
                # Set the current waypoint as the setpoint
                current_waypoint = self.waypoints[self.waypoint_index]
                trajectory_msg.position = [current_waypoint[0], current_waypoint[1], current_waypoint[2]]

                # Move to the next waypoint every 8 seconds
                if self.mission_timer - self.last_state_change_time >= 80:
                    self.waypoint_index += 1
                    if self.waypoint_index < len(self.waypoints):
                         self.get_logger().info(f"Proceeding to waypoint {self.waypoint_index + 1}/{len(self.waypoints)}...")
                    self.last_state_change_time = self.mission_timer
        
        elif self.state == "LANDING":
            if self.mission_timer - self.last_state_change_time >= 50: # After 5 seconds in landing mode
                self.disarm()
                self.state = "LANDED"

        elif self.state == "LANDED":
            self.get_logger().info("Mission complete. Stopping timer.")
            self.timer.cancel()
        
        if self.state != "IDLE":
             self.trajectory_setpoint_publisher.publish(trajectory_msg)

        self.mission_timer += 1

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()