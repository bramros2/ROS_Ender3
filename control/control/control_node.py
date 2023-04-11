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
        # subscribe to keyboard input topic
        self.pumps_initialized = False
        self.pid_subscription = self.create_subscription(Float64(), 'control_signal_topic',  self.pid_input_callback, 1)
        self.feedrate_publisher = self.create_publisher(Float64(), 'current_feedrate', 1)

        self.ser = serial.Serial('/dev/ttyUSB0' , 115200, timeout=1) 
        self.init_pumps(self.ser)
             

    def init_pumps(self,ser):
        setup_gcode = '''
        ; This code is used to setup the pumps in each python script
        ; You can manually edit this, but some settings (like M92) are overwritten later
        ; 31-8-2021 - Bart Grosman
        ;
        ; Setup code from Vittorio
        M201 X500.00 Y500.00 Z500.00 ;Setup machine max acceleration
        M203 X10.0 Y10.0 Z10.00 E50.00 ;Setup machine max feedrate, == speed limit in mm/s
        M204 T500.00 ;Setup Travel acceleration, no print or retract since we are not extruding anything
        M205 X0.40 Y0.40 Z0.40 E5.00 ;Setup Jerk, jerk is the minimum speed that the motors move
        ; This is the most important setting to tune, it defines how many stepper motor steps equate to 1 mm of syringe movement
        M92 X-4000 Y-4000 Z4000 ; 4000 steps per mm, as measured by Vittorio
        M92 X-20000 Y-20000 Z-20000 ; 20000 steps per ml from Barts code
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

        command = 'G1 X{:.5f} Y{:.5f} Z{:.5f} F{:f}\n'.format(0.00167,0.000835,0,0.01) 
        if command != None and self.pumps_initialized == True:
            self.ser.write(command.encode())
            response = self.ser.readline()
            self.get_logger().info('Pumps started with X0.5 Y0.5 Z0.5 F0.01')
            init_feedrate = 0.01
            self.feedpub = Float64()
            print('Inital feedrate = ' + str(init_feedrate))
            self.feedpub.data = init_feedrate
            self.feedrate_publisher.publish(0.01)

    def start_pump(self, feedrate):
        # send Gcode command to move a pump
        command = 'G1 X{:.2f} Y{:.2f} Z{:.2f} F{:f}\n'.format(1.0,0.5,0.5,feedrate)           #TODO: read ratios from settings instead of hardcoding
        if command != None and self.pumps_initialized == True:
            self.ser.write(command.encode())
            self.get_logger().info('New command sent: ' + command)
            response = self.ser.readline()
            self.get_logger().info(response)

    # define PID input callback function
    def pid_input_callback(self,data):
        # get the PID adjusted feedrate
        feedrate_corrected = data.data
        self.start_pump(feedrate_corrected)
        
        
       
# main function
def main():
    # create a node
    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()