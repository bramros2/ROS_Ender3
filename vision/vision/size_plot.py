import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt


class DropletSizeSubscriber(Node):

    def __init__(self):
        super().__init__('droplet_size_subscriber')
        self.line = None
        self.droplet_sizes = []
        self.times = []
        self.init_plot()
        print('Node initialized')

        self.subscription = self.create_subscription(
            Float64,
            'droplet_size',
            lambda msg: self.droplet_size_callback(msg),
            10)
        self.subscription  # prevent unused variable warning

    def init_plot(self):

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.times, self.droplet_sizes)
        plt.show(block = False)

    def droplet_size_callback(self, msg):
        droplet_size = msg.data
        now = self.get_clock().now().to_msg().sec
        self.droplet_sizes.append(droplet_size)
        self.times.append(now)
        print(self.droplet_sizes[-1])

        # Update the plot
        self.line.set_xdata(self.times)
        self.line.set_ydata(self.droplet_sizes)
        self.ax.relim()
        self.ax.autoscale_view
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)

    droplet_subscriber = DropletSizeSubscriber()

    rclpy.spin(droplet_subscriber)

    droplet_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()