import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.get_logger().info("PID node initialized")

        # Initialize PID controller with some parameters
        self.Kp = 0.2  # Proportional gain
        self.Ki = 0.1  # Integral gain
        self.Kd = 0.1  # Derivative gain
        self.min_ref = -10  # Minimum reference value
        self.max_ref = 10  # Maximum reference value
        self.min_output = 0.01  # Minimum output value
        self.max_output = 2.0  # Maximum output value
        self.integral = 0  # Integral term
        self.last_error = 0  # Last error value
        self.last_time = self.get_clock().now()  # Last time the callback was called
        self.feedrate = 0  # Feedrate value
        self.wanted_width = 10  # Desired width of the object to be tracked.                   #TODO: Change to read from settings instead of hardcoded

        # Create a subscription to the feedrate topic
        self.feedrate_subscription = self.create_subscription(
            Float64(),  # Data type of the message received
            'current_feedrate',  # Topic name
            self.feedrate_callback,  # Callback function to handle the received message
            1)  # QoS settings

        # Create a subscription to the image detection
        self.droplet_subscription = self.create_subscription(
            Float64(),  # Data type of the message received
            'droplet_size',  # Topic name
            self.callback,  # Callback function to handle the received message
            1)  # QoS settings

        # Create a publisher for the control signal
        self.publisher = self.create_publisher(
            Float64,  # Data type of the message to be published
            'control_signal_topic',  # Topic name
            1)  # QoS settings
    def feedrate_callback(self, msg):
        self.feedrate = msg.data

    def callback(self, msg):
        # Get the new reference value from the received point message
        current_width = msg.data

        # Calculate the elapsed time since the last update
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        # Calculate the error between the current position and the new reference
        error = self.wanted_width - current_width

        # Calculate the proportional term
        p = self.Kp * error * -1        #-1 due to since width is inversely related to speed, same for I and D term

        # Calculate the integral term
        self.integral += error * dt * -1
        i = self.Ki * self.integral

        # Calculate the derivative term
        derivative = ((error - self.last_error) / dt ) *-1
        d = self.Kd * derivative

        # Calculate the control signal as the sum of the three terms
        control_signal = self.feedrate + p + i + d

        # Limit the control signal to the specified range
        control_signal = max(control_signal, self.min_output)
        control_signal = min(control_signal, self.max_output)
        self.get_logger().info("New feedback rate")
        self.get_logger().info(control_signal)

        # Publish the control signal as a Float64 message
        control_signal_msg = Float64()
        control_signal_msg.data = float(control_signal)
        self.publisher.publish(control_signal_msg)
        self.feedrate = control_signal

        # Update the last error and time for the next iteration
        self.last_error = error
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)

    pid_controller = PIDControllerNode()

    rclpy.spin(pid_controller)

    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()