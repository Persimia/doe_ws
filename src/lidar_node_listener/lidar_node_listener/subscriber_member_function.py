import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt
import numpy as np

from pymavlink import mavutil

from math import copysign

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Setup default values
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.angle_increment = 2 * np.pi / 640
        self.time_incremenet = 0
        self.scan_time = 0
        self.range_min = .08
        self.range_max = 10

        self.theta = np.arange(self.angle_min, self.angle_max, self.angle_increment)

        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.axbackground = None
        self.is_plot_init = False

        # MAVLINK Setup
        self.connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762') # connect to SITL
        self.connection.wait_heartbeat() # wait for a heartbeat
        self.get_logger().info(f"Connected to system: {self.connection.target_system},  component: {self.connection.target_component}")

    def init_plot(self, msg: LaserScan):
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.time_incremenet = msg.time_increment
        self.scan_time = msg.scan_time
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        self.ax.set_rmax(self.range_max)
        self.ax.set_rmin(self.range_min)
        self.ax.set_theta_offset(np.pi / 2.0)
        self.ax.grid(True)

        self.theta = np.arange(self.angle_min, self.angle_max, self.angle_increment)
        r = np.array(msg.ranges)

        
        self.fig.canvas.draw()

        self.axbackground = self.fig.canvas.copy_from_bbox(self.ax.bbox)

        self.line, = self.ax.plot(self.theta,r)
        self.target_line, = self.ax.plot([0, self.theta[0]],[0,r[0]],color='r')
        self.fig.show()

        # mode Guided 
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, # confirmation
            1, # param1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
            29, # param2 (flightmode number)
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

        self.is_plot_init = True

    def find_yaw_error(self, r):
        # r is an array of the ranges in meters
        # self.theta is an array of the azimuths (relative to the vehicle) of each range in radians

        # Aim to the nearest theta
        i = np.argmin(r)
        yaw_error_to_target = self.theta[i]
        self.target_line.set_data([yaw_error_to_target, yaw_error_to_target], [0, r[i]])
        return yaw_error_to_target

    def listener_callback(self, msg):
        if self.is_plot_init == False:
            self.init_plot(msg)
        
        r = np.array(msg.ranges)

        yaw_error_to_target = self.find_yaw_error(r)

        self.fig.canvas.restore_region(self.axbackground)
        self.line.set_data(self.theta, r)
        self.ax.draw_artist(self.line)
        self.ax.draw_artist(self.target_line)

        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()
        
        # Command a yaw
        print(float(np.rad2deg(yaw_error_to_target)))
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            1, # confirmation
            abs(yaw_error_to_target*180/np.pi), # deg
            20, # speed
            copysign(1,-yaw_error_to_target), # dir (1=CW)
            1, # abs (0) or rel (1)
            0, # param5 not used
            0, # param6 not used
            0) # param7 not used

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()