import rclpy
import serial 
import serial.tools.list_ports

from rclpy.node             import Node
from std_msgs.msg           import String

class SerialController(Node):
    def __init__(self):
        super().__init__('manual_control')
        # subscribe to keyboard input topic
        self.pumps_initialized = False
        self.subscription = self.create_subscription(String, '/command_input',  self.keyboard_input_callback, 1)

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

    def start_pump(self,command):
        # send Gcode command to move a pump
        
        if command != None and self.pumps_initialized == True:
            self.ser.write(command.encode())
            response = self.ser.readline()
            self.get_logger().info(response)

    # define keyboard input callback function
    def command_input_callback(self,data):
        # get the pressed key
        commands = data.data
        commands = commands.splitlines()
        for command in commands:
            if command.startswith(';') or command == '\n':
                return
            command_bytes = command.encode() + b'\n'
            self.start_pump(command_bytes)


# main function
def main():
    # create a node
    # start the motor control loop
    rclpy.init()
    key_controller = SerialController()
    rclpy.spin(key_controller)

    key_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()