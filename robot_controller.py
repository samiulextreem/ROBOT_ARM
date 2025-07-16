import serial
import time
import math
import matplotlib.pyplot as plt
import numpy as np

# Import IK function from IK.py
def inverse_kinematics(x, y, L1=100.0, L2=80.0):
    """
    Calculate joint angles for 2-DOF planar robot arm
    
    Args:
        x, y: Target coordinates in millimeters (mm)
        L1: Length of first link in mm (default: 100.0)
        L2: Length of second link in mm (default: 80.0)
    
    Returns:
        (theta1, theta2) in degrees, or None if unreachable
    """
    # Distance to target
    distance = math.sqrt(x*x + y*y)
    
    # Check reachability
    if distance > (L1 + L2) or distance < abs(L1 - L2):
        return None
    
    # Calculate theta2 for both elbow configurations
    cos_theta2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2)
    cos_theta2 = max(-1, min(1, cos_theta2))  # Clamp to valid range
    
    # Two possible solutions for theta2
    theta2_elbow_up = math.acos(cos_theta2)
    theta2_elbow_down = -math.acos(cos_theta2)
    
    solutions = []
    
    # Calculate theta1 for elbow up configuration
    k1_up = L1 + L2 * math.cos(theta2_elbow_up)
    k2_up = L2 * math.sin(theta2_elbow_up)
    theta1_elbow_up = math.atan2(y, x) - math.atan2(k2_up, k1_up)
    solutions.append((theta1_elbow_up, theta2_elbow_up, "elbow_up"))
    
    # Calculate theta1 for elbow down configuration
    k1_down = L1 + L2 * math.cos(theta2_elbow_down)
    k2_down = L2 * math.sin(theta2_elbow_down)
    theta1_elbow_down = math.atan2(y, x) - math.atan2(k2_down, k1_down)
    solutions.append((theta1_elbow_down, theta2_elbow_down, "elbow_down"))
    
    # Choose solution with theta1 that is less negative (more positive)
    best_solution = max(solutions, key=lambda sol: sol[0])
    
    # Convert to degrees
    return math.degrees(best_solution[0]), math.degrees(best_solution[1])

def visualize_robot(x, y, theta1, theta2, L1=100.0, L2=80.0):
    """
    Visualize the robot arm reaching for target position
    """
    # Convert angles to radians
    theta1_rad = math.radians(theta1)
    theta2_rad = math.radians(theta2)
    
    # Calculate joint positions
    base = [0, 0]
    elbow = [L1 * math.cos(theta1_rad), L1 * math.sin(theta1_rad)]
    end_effector = [elbow[0] + L2 * math.cos(theta1_rad + theta2_rad), 
                    elbow[1] + L2 * math.sin(theta1_rad + theta2_rad)]
    
    # Create plot
    plt.figure(figsize=(10, 8))
    
    # Draw workspace (reachable area)
    circle_outer = plt.Circle((0, 0), L1 + L2, fill=False, color='lightblue', linestyle='--', alpha=0.5)
    circle_inner = plt.Circle((0, 0), abs(L1 - L2), fill=False, color='lightblue', linestyle='--', alpha=0.5)
    plt.gca().add_patch(circle_outer)
    plt.gca().add_patch(circle_inner)
    
    # Draw robot arm
    plt.plot([base[0], elbow[0]], [base[1], elbow[1]], 'b-', linewidth=8, label='Link 1 (L1)')
    plt.plot([elbow[0], end_effector[0]], [elbow[1], end_effector[1]], 'r-', linewidth=8, label='Link 2 (L2)')
    
    # Draw joints
    plt.plot(base[0], base[1], 'ko', markersize=12, label='Base Joint')
    plt.plot(elbow[0], elbow[1], 'go', markersize=10, label='Elbow Joint')
    plt.plot(end_effector[0], end_effector[1], 'ro', markersize=10, label='End Effector')
    
    # Draw target
    plt.plot(x, y, 'r*', markersize=15, label='Target')
    
    # Add coordinate annotations
    plt.annotate(f'Base\n(0, 0)', xy=base, xytext=(10, 10), 
                textcoords='offset points', fontsize=9, ha='left')
    plt.annotate(f'Elbow\n({elbow[0]:.1f}, {elbow[1]:.1f})', xy=elbow, xytext=(10, 10), 
                textcoords='offset points', fontsize=9, ha='left')
    plt.annotate(f'Target\n({x}, {y})', xy=(x, y), xytext=(10, 10), 
                textcoords='offset points', fontsize=9, ha='left')
    
    # Add grid and labels
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.xlabel('X (mm)')
    plt.ylabel('Y (mm)')
    plt.title(f'Robot Arm Configuration\nTarget: ({x}mm, {y}mm)\nAngles: θ1={theta1:.1f}°, θ2={theta2:.1f}°')
    plt.legend(loc='upper right')
    
    # Set axis limits
    max_reach = L1 + L2 + 20
    plt.xlim(-max_reach, max_reach)
    plt.ylim(-max_reach, max_reach)
    
    # Add workspace info text
    plt.text(0.02, 0.98, f'Workspace: {abs(L1-L2):.0f}mm to {L1+L2:.0f}mm\nL1: {L1}mm, L2: {L2}mm', 
             transform=plt.gca().transAxes, fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    print(f"📊 Visualization displayed for target ({x}mm, {y}mm)")

# Define your serial ports and baud rate
port1 = 'COM11'  # Change to your actual port for Joint 1 (theta1)
port2 = 'COM4'   # Change to your actual port for Joint 2 (theta2)
baud_rate = 9600  # Common baud rate

# Robot arm parameters in millimeters (mm)
L1 = 100.0  # Length of first link in mm
L2 = 80.0   # Length of second link in mm

def test_robot_serial_ports():
    """
    Test if the configured serial ports can be opened.
    """
    print("=== Testing Robot Serial Ports ===")
    
    # Test PORT1
    try:
        ser1 = serial.Serial(port1, baud_rate, timeout=1)
        print(f"✓ {port1} - OK (can open and close)")
        ser1.close()
    except serial.SerialException as e:
        print(f"✗ {port1} - FAILED: {e}")
    
    # Test PORT2
    try:
        ser2 = serial.Serial(port2, baud_rate, timeout=1)
        print(f"✓ {port2} - OK (can open and close)")
        ser2.close()
    except serial.SerialException as e:
        print(f"✗ {port2} - FAILED: {e}")
    
    print()

def open_robot_serial_ports():
    """
    Opens and returns both serial port connections for the robot.
    """
    print("Opening robot serial ports...")
    
    try:
        ser1 = serial.Serial(port1, baud_rate, timeout=1)
        print(f"✓ Connected to {port1} (Joint 1 - θ1)")
    except serial.SerialException as e:
        print(f"✗ Failed to connect to {port1}: {e}")
        return None, None
    
    try:
        ser2 = serial.Serial(port2, baud_rate, timeout=1)
        print(f"✓ Connected to {port2} (Joint 2 - θ2)")
    except serial.SerialException as e:
        print(f"✗ Failed to connect to {port2}: {e}")
        ser1.close()
        return None, None
    
    # Wait for serial ports to initialize
    time.sleep(2)
    print("✓ Robot serial ports ready")
    print(f"Robot arm parameters: L1={L1}mm, L2={L2}mm")
    print(f"Workspace range: {abs(L1-L2):.1f}mm to {L1+L2:.1f}mm from origin\n")
    
    return ser1, ser2

# Test and open the serial ports
test_robot_serial_ports()
ser1, ser2 = open_robot_serial_ports()

if ser1 is None or ser2 is None:
    print("Failed to open robot serial ports. Exiting.")
    exit()

print("\n=== Robot Arm Position-to-Angle Controller ===")
print("Enter target X,Y coordinates in millimeters (mm) and the system will:")
print("1. Calculate inverse kinematics")
print("2. Send joint angles to serial ports")
print("3. Show real-time visualization")
print("4. Port1 (COM11) = Joint 1 angle (theta1)")
print("5. Port2 (COM4) = Joint 2 angle (theta2)")
print()

try:
    while True:
        # Get target coordinates from user
        try:
            x_input = input("Enter X coordinate in mm (or 'q' to quit): ")
            if x_input.lower() == 'q':
                break
            
            y_input = input("Enter Y coordinate in mm: ")
            if y_input.lower() == 'q':
                break
                
            # Convert to float
            x = float(x_input)
            y = float(y_input)
            
        except ValueError:
            print("Invalid input! Please enter numeric values in mm.")
            continue
        except KeyboardInterrupt:
            break

        print(f"\nTarget position: ({x}mm, {y}mm)")
        
        # Calculate inverse kinematics
        result = inverse_kinematics(x, y, L1, L2)
        
        if result is None:
            print(f"❌ Target ({x}mm, {y}mm) is UNREACHABLE!")
            print(f"Distance: {math.sqrt(x*x + y*y):.2f}mm")
            print(f"Reachable range: {abs(L1-L2):.1f}mm to {L1+L2:.1f}mm")
            print("Please try a different position.\n")
            continue
        
        theta1, theta2 = result
        
        # Send angles to serial ports
        try:
            # Prepare data
            theta1_data = (str(round(theta1, 1)) + '\n').encode()
            theta2_data = (str(round(theta2, 1)) + '\n').encode()
            
            print(f"\nSending joint angles...")
            print(f"Data to {port1}: {theta1_data} (raw bytes: {list(theta1_data)})")
            print(f"Data to {port2}: {theta2_data} (raw bytes: {list(theta2_data)})")
            
            # Send theta1 to port1 and theta2 to port2
            bytes_sent1 = ser1.write(theta1_data)
            bytes_sent2 = ser2.write(theta2_data)
            
            # Flush the output buffers to ensure data is transmitted
            ser1.flush()
            ser2.flush()
            
            print(f"✅ Joint angles calculated and sent:")
            print(f"   θ1 (Joint 1): {theta1:.1f}° → Sent {bytes_sent1} bytes to {port1}")
            print(f"   θ2 (Joint 2): {theta2:.1f}° → Sent {bytes_sent2} bytes to {port2}")
            print(f"   Robot should move to position ({x}mm, {y}mm)")
            
            # Check if there's any data waiting to be sent
            out_waiting1 = ser1.out_waiting
            out_waiting2 = ser2.out_waiting
            print(f"   Bytes still waiting: {port1}={out_waiting1}, {port2}={out_waiting2}")
            
            # Show visualization
            visualize_robot(x, y, theta1, theta2, L1, L2)
            
        except Exception as e:
            print(f"❌ Error sending to serial ports: {e}")
            # Still show visualization even if serial fails
            visualize_robot(x, y, theta1, theta2, L1, L2)
        
        print("-" * 60)

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    # Close ports when exiting
    print("\nClosing robot serial ports...")
    try:
        ser1.close()
        ser2.close()
        print("✓ Robot serial ports closed")
    except:
        pass
    print("Robot controller terminated.")