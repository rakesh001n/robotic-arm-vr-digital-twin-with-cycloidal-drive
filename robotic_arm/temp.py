import serial
import serial.tools.list_ports
import pygame
import time
import sys

def find_esp32():
    """Finds the ESP32 port automatically."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if any(x in port.description for x in ["CP210", "CH340", "USB"]):
            return port.device
    return None

def apply_deadzone(value, threshold=0.1):
    """Filters out small stick drift values near zero."""
    if abs(value) < threshold:
        return 0.0
    return value

def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("❌ No joystick detected. Check your Ant Esports receiver!")
        return

    js = pygame.joystick.Joystick(0)
    js.init()
    
    # FIXED: Pygame 2.x uses get_numaxes()
    num_axes = js.get_numaxes()
    
    print(f"🎮 Controller: {js.get_name()}")
    print(f"📊 Detected {num_axes} axes.")

    port_path = find_esp32()
    if not port_path:
        print("❌ ESP32 not found.")
        return

    try:
        # Using context manager to ensure port release
        with serial.Serial(port_path, 115200, timeout=0.1) as ser:
            time.sleep(1) 
            print(f"🚀 Connected to {port_path}. Sending stick data...")

            while True:
                pygame.event.pump()
                
                # Reading Left Stick (Axes 0 and 1)
                lx = apply_deadzone(round(js.get_axis(0), 2))
                ly = apply_deadzone(round(js.get_axis(1), 2))
                
                # Reading Right Stick (Commonly axes 2, 3, or 4 depending on Mode)
                # For Ant Esports/XInput, Right Stick is usually 3 and 4
                rx = apply_deadzone(round(js.get_axis(3), 2)) if num_axes > 3 else 0.0
                ry = apply_deadzone(round(js.get_axis(4), 2)) if num_axes > 4 else 0.0
                
                # Format: "LX,LY|RX,RY\n"
                msg = f"{lx},{ly}|{rx},{ry}\n"
                
                # Send to ESP32
                ser.write(msg.encode('utf-8'))
                
                # Terminal Feedback (Single line update)
                output = f"L: [{lx:>5}, {ly:>5}] | R: [{rx:>5}, {ry:>5}]"
                sys.stdout.write(f"\r{output}      ")
                sys.stdout.flush()

                time.sleep(0.05) # 20Hz frequency

    except serial.SerialException as e:
        print(f"\n❌ Serial Error: {e}")
        print("💡 Ensure Serial Monitor is CLOSED and run: sudo fuser -k /dev/ttyUSB0")
    except KeyboardInterrupt:
        print("\n👋 Stopping...")
    finally:
        pygame.quit()
        print("✅ Port released and Pygame closed.")

if __name__ == "__main__":
    main()