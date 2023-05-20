from autoware_auto_control_msgs.msg import AckermannControlCommand
from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from matplotlib import pyplot as plt

class LongitudinalPlotter(Node):
    def __init__(self) -> None:
        super().__init__('longitudinal_control_plot')
        
        self._vehicle_control_command_subscriber = self.create_subscription(
            CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd',
            self._vehicle_control_command_callback, 1)
        self.throttle_values = []
        self.brake_values = []
        self.control_command_timestamps = []

        self._ackermann_control_command_subscriber = self.create_subscription(
            AckermannControlCommand, '/control/command/control_cmd',
            self._ackermann_control_command_callback, 1)
        self.acc_setpoint_values = []
        self.vel_setpoint_values = []
        self.control_setpoint_timestamps = []

        self._acceleration_subscriber = self.create_subscription(
            AccelWithCovarianceStamped, '/localization/acceleration',
            self._acceleration_callback, 1)
        self.acc_process_values = []
        self.acc_process_timestamps = []

        self._odometry_subscriber = self.create_subscription(
            Odometry, '/localization/kinematic_state',
            self._odometry_callback, 1)
        self.vel_process_values = []
        self.vel_process_timestamps = []
    
    def show(self):
        figure, axes = plt.subplots(2,2)
        axes = axes.ravel()

        axes[0].plot(self.control_setpoint_timestamps, self.acc_setpoint_values, '-r', label='setpoint', lw=2)
        axes[0].plot(self.acc_process_timestamps, self.acc_process_values, '-g', label='process', lw=2)
        axes[0].set_title("acceleration")
        axes[0].legend(loc="upper right")
        
        axes[1].plot(self.control_command_timestamps, self.brake_values, '-r', label='brake', lw=2)
        axes[1].plot(self.control_command_timestamps, self.throttle_values, '-g', label='throttle', lw=2)
        axes[1].set_title("actuation")
        axes[1].legend(loc="upper right")

        axes[2].plot(self.control_setpoint_timestamps, self.vel_setpoint_values, '-r', label='setpoint', lw=2)
        axes[2].plot(self.vel_process_timestamps, self.vel_process_values, '-g', label='process', lw=2)
        axes[2].set_title("velocity")
        axes[2].legend(loc="upper right")

        plt.legend()
        plt.show()
    
    def _vehicle_control_command_callback(self, vehicle_control_command_msg):
        timestamp = vehicle_control_command_msg.header.stamp.sec + vehicle_control_command_msg.header.stamp.nanosec / 1e9
        throttle = vehicle_control_command_msg.throttle
        brake = vehicle_control_command_msg.brake
        print(f'{timestamp}) throttle: {throttle}')
        print(f'{timestamp}) brake: {brake}')
        self.control_command_timestamps.append(timestamp)
        self.throttle_values.append(throttle)
        self.brake_values.append(brake)
    
    def _ackermann_control_command_callback(self, ackermann_control_command_msg):
        timestamp = ackermann_control_command_msg.stamp.sec + ackermann_control_command_msg.stamp.nanosec / 1e9
        acc_setpoint = ackermann_control_command_msg.longitudinal.acceleration
        vel_setpoint = ackermann_control_command_msg.longitudinal.speed
        print(f'{timestamp}) accel setpoint: {acc_setpoint}')
        print(f'{timestamp}) vel setpoint: {vel_setpoint}')
        self.control_setpoint_timestamps.append(timestamp)

        self.vel_setpoint_values.append(vel_setpoint)
        self.acc_setpoint_values.append(acc_setpoint)
        # ackermann_control_command_msg.longitudinal.jerk
        
    def _acceleration_callback(self, acceleration_msg):
        timestamp = acceleration_msg.header.stamp.sec + acceleration_msg.header.stamp.nanosec / 1e9
        acc_process = acceleration_msg.accel.accel.linear.x
        print(f'{timestamp}) accel process: {acc_process}')
        self.acc_process_timestamps.append(timestamp)
        self.acc_process_values.append(acc_process)

    def _odometry_callback(self, odometry_msg):
        timestamp = odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nanosec / 1e9
        vel_process = odometry_msg.twist.twist.linear.x
        print(f'{timestamp}) vel process: {vel_process}')
        self.vel_process_timestamps.append(timestamp)
        self.vel_process_values.append(vel_process)

def main():
    rclpy.init()
    long_plotter = LongitudinalPlotter()

    try:
        rclpy.spin(long_plotter)
    except KeyboardInterrupt:
        pass

    long_plotter.show()
    long_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
