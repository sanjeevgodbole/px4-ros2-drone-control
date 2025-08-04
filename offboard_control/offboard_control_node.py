import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, SensorGps
from rclpy.clock import Clock
import math

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode with GPS memory."""

    def __init__(self):
        super().__init__('offboard_control_gps_mission')

        # Configure QoS Profiles
        command_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Create Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', command_qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', command_qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', command_qos_profile)
        
        # Create Subscribers
        self.odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, sensor_qos_profile)
        self.gps_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.gps_callback, sensor_qos_profile)

        # Define the flight path with a final "Return to Launch" waypoint
        self.waypoints = [
            [0.0, 100.0, -10.0],     # 1. Fly 100m North
            [50.0, 100.0, -10.0],    # 2. Fly 50m East
            [50.0, -100.0, -10.0],   # 3. Fly 200m South
            [-50.0, -100.0, -10.0],  # 4. Fly 100m West
            [-50.0, 100.0, -10.0],   # 5. Fly 200m North
            [0.0, 100.0, -10.0],     # 6. Fly 50m East
            [0.0, 0.0, -10.0]        # 7. Return to origin before landing
        ]
        
        # Initialize variables
        self.state = "IDLE"
        self.current_pos = [0.0, 0.0, 0.0]
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.saved_gps_position = None
        self.origin_gps_saved = False
        self.waypoint_index = 0
        self.acceptance_radius = 5.0
        self.mission_timer = 0

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("GPS-enabled return-to-launch mission node has been started.")

    def odometry_callback(self, msg):
        """Update the drone's current local position."""
        self.current_pos = msg.position

    def gps_callback(self, msg):
        """Update the drone's current GPS position."""
        self.current_latitude = msg.latitude_deg
        self.current_longitude = msg.longitude_deg

    def remember_gps_position(self):
        """Saves the current GPS coordinates."""
        if self.current_latitude != 0.0:
            self.saved_gps_position = (self.current_latitude, self.current_longitude)
            self.get_logger().info(f"Saved origin GPS position: Lat {self.saved_gps_position[0]}, Lon {self.saved_gps_position[1]}")
        else:
            self.get_logger().warn("Could not save GPS position, no data received yet.")

    def get_distance_to_waypoint(self): # ... (same as before)
        if self.waypoint_index >= len(self.waypoints): return float('inf')
        target_pos = self.waypoints[self.waypoint_index]
        return math.sqrt(
            (self.current_pos[0] - target_pos[0])**2 +
            (self.current_pos[1] - target_pos[1])**2 +
            (self.current_pos[2] - target_pos[2])**2)

    def publish_vehicle_command(self, command, **params): # ... (same as before)
        msg = VehicleCommand()
        msg.command, msg.param1, msg.param2 = command, params.get("param1", 0.0), params.get("param2", 0.0)
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 1, 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0); self.get_logger().info('Arm command sent')
    def disarm(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0); self.get_logger().info('Disarm command sent')
    def engage_offboard_mode(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0); self.get_logger().info("Switching to Offboard mode")
    def land(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND); self.get_logger().info("Switching to land mode")
        
    def timer_callback(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position, offboard_msg.velocity, offboard_msg.acceleration, offboard_msg.attitude, offboard_msg.body_rate = True, False, False, False, False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(offboard_msg)

        trajectory_msg = TrajectorySetpoint()

        if self.state == "IDLE":
            if self.mission_timer == 10:
                self.engage_offboard_mode()
                self.arm()
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            trajectory_msg.position = [0.0, 0.0, -10.0]
            self.trajectory_setpoint_publisher.publish(trajectory_msg)
            if abs(self.current_pos[2] + 10.0) < 1.0:
                if not self.origin_gps_saved:
                    self.remember_gps_position()
                    self.origin_gps_saved = True
                self.get_logger().info("Takeoff complete. Starting waypoint mission.")
                self.state = "FLYING_WAYPOINTS"
        
        elif self.state == "FLYING_WAYPOINTS":
            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().info("All waypoints reached. Landing.")
                self.land()
                self.state = "LANDING"
            else:
                target_pos = self.waypoints[self.waypoint_index]
                trajectory_msg.position = [target_pos[0], target_pos[1], target_pos[2]]
                self.trajectory_setpoint_publisher.publish(trajectory_msg)

                if self.get_distance_to_waypoint() < self.acceptance_radius:
                    self.get_logger().info(f"Arrived at waypoint {self.waypoint_index + 1}.")
                    self.waypoint_index += 1
        
        elif self.state == "LANDING":
            if self.mission_timer > 1200:
                self.disarm()
                self.state = "LANDED"

        elif self.state == "LANDED":
            self.get_logger().info("Mission complete.")
            self.timer.cancel()

        self.mission_timer += 1

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()