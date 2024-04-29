# SWIRV Controller Command Code
# Code uses example modified from by adding serial output
# https://www.pygame.org/docs/ref/joystick.html?highlight=squar

# to use run pip3 to install all dependencies

import serial
import serial.tools.list_ports
import time
import atexit
import pygame

# code for finding which port ESP32 is connected to, manufacturer is "wch.cn" for C6
# if using different device run Port_info
# "Silicon Labs" for Esp32 C6
# https://justinbois.github.io/bootcamp/2020/lessons/l39_serial.html
def find_ESP32(port=None):
    if port is None:
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.manufacturer is not None and "Silicon Labs" in p.manufacturer:
                port = p.device
    return port

# connect to ESP32
port = find_ESP32()
ESP32 = serial.Serial(port, baudrate=115200, timeout=1)
# ensure port is clear after initial handshake
ESP32.close()
ESP32.open()
# define function for initializing connection
def handshake_ESP32(ESP32, sleep_time=1):
    time.sleep(sleep_time)
    timeout = ESP32.timeout
    ESP32.timeout = 2

    ESP32.write(bytes([1]))
    while (ESP32.in_waiting < 0):
        pass
    _ = ESP32.read_until()
    ESP32.timeout = timeout
# run handshape code to initialize the connection
handshake_ESP32(ESP32)
ESP32.close()
with serial.Serial(port, baudrate=115200, timeout=1) as ESP32:
    handshake_ESP32(ESP32)
# setup connection object
def open_ESP32(port, baudrate=115200, timeout=1):
    ESP32 = serial.Serial(port, baudrate=baudrate, timeout=timeout)
    ESP32.close()
    ESP32.open()
    handshake_ESP32(ESP32)
    return ESP32
ESP32 = open_ESP32(port)

pygame.init()
# This is a simple class that will help us print to the screen.
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 25)

    def tprint(self, screen, text):
        text_bitmap = self.font.render(text, True, (0, 0, 0))
        screen.blit(text_bitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


def main():
    # Set the width and height of the screen (width, height), and name the window.
    screen = pygame.display.set_mode((480, 640))
    pygame.display.set_caption("SWIRV Joystick Control")
    # Used to manage how fast the screen updates.
    clock = pygame.time.Clock()
    # Get ready to print.
    text_print = TextPrint()
    # This dict can be left as-is, since pygame will generate a
    # pygame.JOYDEVICEADDED event for every joystick connected
    # at the start of the program.
    joysticks = {}
    # set control modes for ESP32 and Jeton Nano
    # wifi_mode controls whether the Jetson is hosting an network or connecting to guest wifi button Y
    # control_mode sets whether the system is running on ROS or from manual control, button A
    wifi_mode = 0
    control_mode = 0
    boost_mode = 0

    done = False
    while not done:
        # Event processing step.
        # Possible joystick events: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
        # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                ESP32.close()
                done = True  # Flag that we are done so we exit this loop.

            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
                # test ESP
                if (event.button == 3 and wifi_mode == 0):
                    wifi_mode = 1
                elif (event.button == 3 and wifi_mode == 1):
                   wifi_mode = 0
                if (event.button == 0 and control_mode == 0):
                    control_mode = 1
                elif (event.button == 0 and control_mode == 1):
                    control_mode = 0
                if (event.button == 2 and boost_mode == 0):
                    boost_mode = 1
                elif (event.button == 2 and boost_mode == 1):
                    boost_mode = 0

            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")

            # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                # This event will be generated when the program starts for every
                # joystick, filling up the list without needing to create them manually.
                joy = pygame.joystick.Joystick(event.device_index)
                joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} connencted")

            if event.type == pygame.JOYDEVICEREMOVED:
                del joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} disconnected")

        # Drawing step
        # First, clear the screen to white. Don't put other drawing commands
        # above this, or they will be erased with this command.
        screen.fill((255, 255, 255))
        text_print.reset()

        # Get count of joysticks.
        joystick_count = pygame.joystick.get_count()

        text_print.tprint(screen, f"Number of joysticks: {joystick_count}")
        text_print.indent()

        # For each joystick:
        for joystick in joysticks.values():
            jid = joystick.get_instance_id()

            text_print.tprint(screen, f"Joystick {jid}")
            text_print.indent()

            # Get the name from the OS for the controller/joystick.
            name = joystick.get_name()
            text_print.tprint(screen, f"Joystick name: {name}")

            guid = joystick.get_guid()
            text_print.tprint(screen, f"GUID: {guid}")

            power_level = joystick.get_power_level()
            text_print.tprint(screen, f"Joystick's power level: {power_level}")

            # Usually axis run in pairs, up/down for one, and left/right for
            # the other. Triggers count as axes.
            axes = joystick.get_numaxes()
            text_print.tprint(screen, f"Number of axes: {axes}")
            text_print.indent()

            for i in range(axes):
                axis = joystick.get_axis(i)
                # put Serial output for axis 1 and 3 -1 to 1 for forward/reverse
                left = joystick.get_axis(1)
                right = joystick.get_axis(3)
                # convert floats in range (-1.000 to 1.000) to ints in range (0 to 2000)
                if boost_mode == 0:
                    int_left = int(left*-500)
                    int_right = int(right*-500)
                elif boost_mode == 1:
                    int_left = int(left*-1000)
                    int_right = int(right*-1000)
                # convert ints into strings of correct length
                s_left = format(int_left, '05d')
                s_right = format(int_right, '05d')
                # encode movement as a 10 digit byte array in format s12341234e
                move_message= s_left+s_right
                
                text_print.tprint(screen, f"Axis {i} value: {axis:>6.3f}")
            text_print.unindent()

            buttons = joystick.get_numbuttons()
            text_print.tprint(screen, f"Number of buttons: {buttons}")
            text_print.indent()

            for i in range(buttons):
                button = joystick.get_button(i)
                text_print.tprint(screen, f"Button {i:>2} value: {button}")


            text_print.unindent()

            hats = joystick.get_numhats()
            text_print.tprint(screen, f"Number of hats: {hats}")
            text_print.indent()

            # Hat position. All or nothing for direction, not a float like
            # get_axis(). Position is a tuple of int values (x, y).
            for i in range(hats):
                hat = joystick.get_hat(i)
                text_print.tprint(screen, f"Hat {i} value: {str(hat)}")
            text_print.unindent()

            text_print.unindent()
            

        # write status of control modes
        if control_mode == 0:
            ctr_mode = "Manual Control Mode"
        elif control_mode == 1:
            ctr_mode = "ROS Control Mode"
        if wifi_mode == 1:
            net_mode = "Local access point"
        elif wifi_mode == 0:
            net_mode = "WCU Network"
        if boost_mode == 1:
            speed_mode = "Boost mode"
        elif boost_mode == 0:
            speed_mode = "Normal mode"
            
        text_print.tprint(screen, f"Wi-Fi status:    {net_mode}")
        text_print.tprint(screen, f"Control Access:    {ctr_mode}")
        text_print.tprint(screen, f"Speed:    {speed_mode}")

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()
        # Limit to 50 frames per second, same as speed controller update frequency
        clock.tick(50)
        control_message = ('s' + move_message + (format(wifi_mode, '01d')) + (format(control_mode, '01d')) + 'e').encode()
        print(control_message)
        ESP32.write(bytes(control_message))


if __name__ == "__main__":
    main()
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()
