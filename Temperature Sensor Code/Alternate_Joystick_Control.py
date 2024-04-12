import serial
import serial.tools.list_ports
import time
import pygame

# Function to find the ESP32 device
def find_ESP32():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if p.manufacturer and "Silicon Labs" in p.manufacturer:
            return p.device
    return None

# Initialize the ESP32 connection
def init_ESP32(port, baudrate=115200, timeout=1):
    try:
        ESP32 = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        ESP32.write(b'1')  # Send initial handshake
        time.sleep(1)
        while ESP32.in_waiting == 0:
            pass
        ESP32.read()  # Read the response
        print(f"Connected to ESP32 on port: {port}")
        return ESP32
    except Exception as e:
        print(f"Failed to initialize ESP32 connection: {e}")
        return None

# Handle joystick inputs and send motor commands to ESP32
def handle_joystick_input(joystick, ESP32):
    # Implement dead zones
    dead_zone = 0.05
    axis_threshold = 0.1
    
    left = joystick.get_axis(1)
    right = joystick.get_axis(3)
    
    # Apply dead zones
    if abs(left) < dead_zone:
        left = 0
    if abs(right) < dead_zone:
        right = 0
    
    # Only send commands if inputs exceed the threshold
    if abs(left) > axis_threshold or abs(right) > axis_threshold:
        # Convert floats to integers in the range of -1000 to 1000
        int_left = int(left * 1000)
        int_right = int(right * 1000)
        
        # Create motor command messages
        control_motor_left(int_left, ESP32)
        control_motor_right(int_right, ESP32)
        
        print(f"Left motor speed: {int_left}, Right motor speed: {int_right}")

# Functions to control each motor
def control_motor_left(speed, ESP32):
    # Convert speed value to appropriate format for ESP32
    command = f"s{speed:05d}e"
    ESP32.write(command.encode())

def control_motor_right(speed, ESP32):
    # Convert speed value to appropriate format for ESP32
    command = f"r{speed:05d}e"
    ESP32.write(command.encode())

# Display joystick information on the Pygame screen
def display_joystick_info(screen, joysticks):
    screen.fill((255, 255, 255))
    y_offset = 10
    font = pygame.font.Font(None, 25)
    
    for jid, joystick in joysticks.items():
        name = joystick.get_name()
        num_axes = joystick.get_numaxes()
        num_buttons = joystick.get_numbuttons()
        num_hats = joystick.get_numhats()
        
        info_text = f"Joystick {jid}: {name}"
        info_text += f"\nAxes: {num_axes}, Buttons: {num_buttons}, Hats: {num_hats}"
        text_surface = font.render(info_text, True, (0, 0, 0))
        screen.blit(text_surface, (10, y_offset))
        y_offset += 50
    
    pygame.display.update()

# Main function
def main():
    pygame.init()
    
    # Set up the screen
    screen = pygame.display.set_mode((450, 450))
    pygame.display.set_caption("SWIRV Joystick Control")
    
    # Find and initialize ESP32
    port = find_ESP32()
    if port is None:
        print("ESP32 not found.")
        return
    
    ESP32 = init_ESP32(port)
    if ESP32 is None:
        return
    
    # Initialize joystick dictionary
    joysticks = {}
    
    running = True
    while running:
        try:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.JOYBUTTONDOWN:
                    print("Joystick button pressed.")
                elif event.type == pygame.JOYBUTTONUP:
                    print("Joystick button released.")
                elif event.type == pygame.JOYDEVICEADDED:
                    # Add new joystick
                    joystick = pygame.joystick.Joystick(event.device_index)
                    joystick.init()
                    joysticks[joystick.get_instance_id()] = joystick
                    print(f"Joystick {joystick.get_instance_id()} connected.")
                elif event.type == pygame.JOYDEVICEREMOVED:
                    # Remove disconnected joystick
                    instance_id = event.instance_id
                    if instance_id in joysticks:
                        del joysticks[instance_id]
                        print(f"Joystick {instance_id} disconnected.")
            
            # Handle joystick inputs and send commands to ESP32
            for joystick in joysticks.values():
                handle_joystick_input(joystick, ESP32)
            
            # Display joystick information on the Pygame screen
            display_joystick_info(screen, joysticks)
            
            # Delay to limit the frame rate
            pygame.time.delay(30)
        
        except Exception as e:
            print(f"Error occurred: {e}")
            running = False
    
    # Clean up and close connections
    ESP32.close()
    pygame.quit()

# Entry point
if __name__ == "__main__":
    main()
