import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt
import numpy as np

# sensor_msgs.msg.LaserScan(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=172, nanosec=500000000),
#  frame_id='base_scan'), angle_min=-3.1415927410125732, angle_max=3.1415927410125732, angle_increment=0.009832840412855148,
#  time_increment=0.0, scan_time=0.0, range_min=0.07999999821186066, range_max=10.0,

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
        self.angle_increment = 2 * np.pi / 639
        self.time_incremenet = 0
        self.scan_time = 0
        self.range_min = .08
        self.range_max = 10

        self.theta = np.arange(self.angle_min, self.angle_max, self.angle_increment)

        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.axbackground = None
        self.is_plot_init = False

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
        self.ax.grid(True)

        self.theta = np.arange(self.angle_min, self.angle_max, self.angle_increment)
        r = np.array(msg.ranges)

        self.line, = self.ax.plot(self.theta,r)
        self.fig.canvas.draw()

        self.axbackground = self.fig.canvas.copy_from_bbox(self.ax.bbox)

        self.fig.show()

        self.is_plot_init = True

    def listener_callback(self, msg):
        if self.is_plot_init == False:
            self.init_plot(msg)
        
        r = np.array(msg.ranges)

        self.fig.canvas.restore_region(self.axbackground)
        self.line.set_data(self.theta, r)
        self.ax.draw_artist(self.line)

        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()
        # self.get_logger().info('I heard: "%s"' % msg.data)

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