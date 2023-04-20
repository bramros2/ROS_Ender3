import rclpy
import serial 
import serial.tools.list_ports
from rclpy.node             import Node
from std_msgs.msg           import String
from std_msgs.msg           import Float64


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info("Control node initialized")
        self.pumps_initialized = False

        # Initialize PID controller with some parameters
        self.Kp = 0.2  # Proportional gain
        self.Ki = 0.1  # Integral gain
        self.Kd = 0.0  # Derivative gain
        self.min_ref = -10  # Minimum reference value
        self.max_ref = 10  # Maximum reference value
        self.min_output = 0.01  # Minimum output value
        self.max_output = 2.0  # Maximum output value
        self.integral = 0  # Integral term
        self.last_error = 0  # Last error value
        self.last_time = self.get_clock().now()  # Last time the callback was called
        self.feedrate = 0.05  # Feedrate value
        self.ratio = 0.2      #ratio between x/y
        self.measured = None
        self.sec_per_update = 30

        
        self.wanted_width = 20  # Desired width of the object to be tracked.                   #TODO: Change to read from settings instead of hardcoded

        # Create a subscription to the image detection
        self.droplet_subscription = self.create_subscription(
            Float64(),  # Data type of the message received
            'droplet_size',  # Topic name
            self.callback,  # Callback function to handle the received message
            1)  # QoS settings

        # Open a serial connection
        self.ser = serial.Serial('/dev/ttyUSB0' , 115200, timeout=1) 

        # Initialize the Pumps with GCode settings
        self.init_pumps(self.ser)

    def callback(self,msg):

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt < self.sec_per_update:
            return
        self.measured = msg.data

        # Calculate the error between the current position and the new reference
        error = self.wanted_width - self.measured

        # Calculate the proportional term
        p = self.Kp * error * -1        #-1 due to since width is inversely related to speed, same for I and D term

        # Calculate the integral term
        self.integral += error * dt * -1
        i = self.Ki * self.integral

        # Calculate the derivative term
        derivative = ((error - self.last_error) / dt ) *-1
        d = self.Kd * derivative

        # Calculate the control signal as the sum of the three terms
        control_signal = self.ratio + p + i + d

        # Limit the control signal to the specified range
        control_signal = max(control_signal, self.min_output)
        control_signal = min(control_signal, self.max_output)
        self.get_logger().info("New feedback rate")
        self.get_logger().info(control_signal)

        self.ratio = control_signal
        print(control_signal)
        #self.start_pump(control_signal)
        # Update the last error and time for the next iteration
        self.last_error = error
        self.last_time = now


    def init_pumps(self,ser):
        setup_gcode = '''

        ; Setup code from Vittorio
        M201 X500.00 Y500.00 Z500.00 ;Setup machine max acceleration
        M203 X10.0 Y10.0 Z10.00 E50.00 ;Setup machine max feedrate, == speed limit in mm/s
        M204 T500.00 ;Setup Travel acceleration, no print or retract since we are not extruding anything
        M205 X0.40 Y0.40 Z0.40 E5.00 ;Setup Jerk, jerk is the minimum speed that the motors move
        ; This is the most important setting to tune, it defines how many stepper motor steps equate to 1 mm of syringe movement
        M92 X-20000 Y-20000 Z20000 ; 20000 steps per ml from Barts code
        M302 S0 ; print with cold hotend -- This allows cold extrusion, but we aren't doing any, maybe when we use the stepper motor of the extruder too
        M121 ; don't use endstops
        G91 ; relative positioning'''

        command_lines = setup_gcode.splitlines()
        for command in command_lines:
            if command.startswith(';') or command == '\n':
                return
            command_bytes = command.encode() + b'\n'
            ser.write(command_bytes)
            response = ser.readline()
            self.get_logger().info('Serial response:')
            self.get_logger().info(response)
        self.pumps_initialized = True
        self.get_logger().info('Ender3 initialised')


    def start_pump(self, ratio):
        # send Gcode command to move a pump
        if ratio < 1:
            x = (self.sec_per_update/60)*self.feedrate
            y = x/ratio
        else: 
            y = (self.sec_per_update/60)*self.feedrate
            x = y*ratio

        command = 'G1 X{:.4f} Y{:.4f} Z{:.4f} F{:f}\n'.format(x,y,0,self.feedrate)           #TODO: read ratios from settings instead of hardcoding
        if command != None and self.pumps_initialized == True:
            self.ser.write(command.encode())
            self.get_logger().info('New command sent: ' + command)
            response = self.ser.readline()
            self.get_logger().info(response)


def main(args=None):
    rclpy.init(args=args)

    pid_controller = Controller()

    rclpy.spin(pid_controller)

    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()