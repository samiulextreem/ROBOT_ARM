import serial
import time
import math
import matplotlib.pyplot as plt
import numpy as np

# ===== ROBOT ARM CONFIGURATION =====
# Serial Port Settings
PORT1 = 'COM11'   # Port for Joint 1 (theta1)
PORT2 = 'COM10'  # Port for Joint 2 (theta2)
BAUD_RATE = 9600 # Communication speed
TIMEOUT = 1      # Timeout in seconds

# Robot Arm Physical Parameters (in millimeters)
L1 = 214.0  # Length of first link in mm
L2 = 184.0   # Length of second link in mm

# Robot Direction Configuration
# Set to True if positive angles should rotate clockwise (opposite of math convention)
CLOCKWISE_POSITIVE = False  # Change to True if your robot follows clockwise-positive convention

# Initial Robot Position (both links upright, pointing upwards)
INITIAL_THETA1 = 0.0  # degrees, robot convention (upright)
INITIAL_THETA2 = 0.0   # degrees, robot convention
# =====================================

# Import IK function from IK.py
def inverse_kinematics(x, y, L1=L1, L2=L2):
    """
    Calculate joint angles for 2-DOF planar robot arm
    
    Args:
        x, y: Target coordinates in millimeters (mm)
        L1: Length of first link in mm (default: 240.0)
        L2: Length of second link in mm (default: 180.0)
    
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
    
    # Convert to degrees and return
    theta1_deg = math.degrees(best_solution[0])
    theta2_deg = math.degrees(best_solution[1])
    
    # Convert from math convention (0°=right) to robot convention (0°=up)
    # The 90° offset is ALWAYS needed because robot zero = UP, math zero = RIGHT
    if CLOCKWISE_POSITIVE:
        # Robot uses clockwise as positive direction
        # robot_θ1 = 90° - math_θ1
        # robot_θ2 = -math_θ2
        theta1_deg = 90 - theta1_deg
        theta2_deg = -theta2_deg
        direction_str = "CW+"
    else:
        # Robot uses counter-clockwise as positive direction (like math, but offset)
        # robot_θ1 = math_θ1 - 90°
        # robot_θ2 = -math_θ2 (Joint 2 motor is reversed)
        theta1_deg = theta1_deg - 90
        theta2_deg = -theta2_deg
        direction_str = "CCW+"
    
    # Debug output for theta calculations
    print(f"🔧 IK Debug: cos_theta2={cos_theta2:.4f}")
    print(f"🔧 IK Debug: theta2_elbow_up={math.degrees(theta2_elbow_up):.2f}°, theta2_elbow_down={math.degrees(theta2_elbow_down):.2f}°")
    print(f"🔧 IK Debug: Selected solution: {best_solution[2]}")
    print(f"🔧 IK Debug: Robot angles (0°=UP, {direction_str}): θ1={theta1_deg:.2f}°, θ2={theta2_deg:.2f}°")
    
    return theta1_deg, theta2_deg

def visualize_robot(x, y, theta1, theta2, L1=L1, L2=L2):
    """
    Visualize the robot arm reaching for target position
    Note: theta1, theta2 are in robot convention (0°=UP, CW+)
          Visualization uses standard math convention (0°=RIGHT, CCW+)
    """
    # Convert from robot convention (0°=UP) back to math convention (0°=RIGHT, CCW+)
    if CLOCKWISE_POSITIVE:
        # Inverse of: robot_θ1 = 90 - math_θ1 → math_θ1 = 90 - robot_θ1
        # Inverse of: robot_θ2 = -math_θ2 → math_θ2 = -robot_θ2
        viz_theta1 = 90 - theta1
        viz_theta2 = -theta2
    else:
        # Inverse of: robot_θ1 = math_θ1 - 90 → math_θ1 = robot_θ1 + 90
        # Inverse of: robot_θ2 = -math_θ2 → math_θ2 = -robot_θ2
        viz_theta1 = theta1 + 90
        viz_theta2 = -theta2
    
    # Convert angles to radians for math calculations
    theta1_rad = math.radians(viz_theta1)
    theta2_rad = math.radians(viz_theta2)
    
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
    plt.title(f'Robot Arm Configuration\nTarget: ({x}mm, {y}mm)\nRobot Angles: θ1={theta1:.1f}°, θ2={theta2:.1f}° (Clockwise+)' if CLOCKWISE_POSITIVE else f'Robot Arm Configuration\nTarget: ({x}mm, {y}mm)\nAngles: θ1={theta1:.1f}°, θ2={theta2:.1f}°')
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

def test_robot_serial_ports():
    """
    Test if the configured serial ports can be opened.
    """
    print("=== Testing Robot Serial Ports ===")
    
    # Test PORT1
    try:
        ser1 = serial.Serial(PORT1, BAUD_RATE, timeout=TIMEOUT)
        print(f"✓ {PORT1} - OK (can open and close)")
        ser1.close()
    except serial.SerialException as e:
        print(f"✗ {PORT1} - FAILED: {e}")
    
    # Test PORT2
    try:
        ser2 = serial.Serial(PORT2, BAUD_RATE, timeout=TIMEOUT)
        print(f"✓ {PORT2} - OK (can open and close)")
        ser2.close()
    except serial.SerialException as e:
        print(f"✗ {PORT2} - FAILED: {e}")
    
    print()

def open_robot_serial_ports():
    """
    Opens and returns both serial port connections for the robot.
    """
    print("Opening robot serial ports...")
    
    try:
        ser1 = serial.Serial(PORT1, BAUD_RATE, timeout=TIMEOUT)
        print(f"✓ Connected to {PORT1} (Joint 1 - θ1)")
    except serial.SerialException as e:
        print(f"✗ Failed to connect to {PORT1}: {e}")
        return None, None
    
    try:
        ser2 = serial.Serial(PORT2, BAUD_RATE, timeout=TIMEOUT)
        print(f"✓ Connected to {PORT2} (Joint 2 - θ2)")
    except serial.SerialException as e:
        print(f"✗ Failed to connect to {PORT2}: {e}")
        ser1.close()
        return None, None
    
    # Wait for serial ports to initialize
    time.sleep(2)
    print("✓ Robot serial ports ready")
    print(f"Robot arm parameters: L1={L1}mm, L2={L2}mm")
    print(f"Workspace range: {abs(L1-L2):.1f}mm to {L1+L2:.1f}mm from origin\n")
    
    return ser1, ser2

def send_angles(ser1, ser2, theta1, theta2):
    """
    Send joint angles to the robot serial ports.
    theta1, theta2 should be in robot convention (degrees).
    """
    theta1_rounded = round(theta1, 1)
    theta2_rounded = round(theta2, 1)
    
    theta1_data = (str(theta1_rounded) + '\n').encode()
    theta2_data = (str(theta2_rounded) + '\n').encode()
    
    ser1.write(theta1_data)
    ser2.write(theta2_data)
    ser1.flush()
    ser2.flush()
    
    print(f"✓ Sent angles: θ1={theta1_rounded}°, θ2={theta2_rounded}°")

# Test and open the serial ports
test_robot_serial_ports()
ser1, ser2 = open_robot_serial_ports()

if ser1 is None or ser2 is None:
    print("Failed to open robot serial ports. Exiting.")
    exit()

# Set initial position (both links upright)
print("Setting initial robot position (upright, angles 0°, 0°)...")
send_angles(ser1, ser2, INITIAL_THETA1, INITIAL_THETA2)
print("✓ Initial position set\n")

print("\n=== Robot Arm Position-to-Angle Controller ===")
print("Enter target X,Y coordinates in millimeters (mm) and the system will:")
print("1. Calculate inverse kinematics")
print("2. Send joint angles to serial ports")
print("3. Show real-time visualization")
print(f"4. Port1 ({PORT1}) = Joint 1 angle (theta1)")
print(f"5. Port2 ({PORT2}) = Joint 2 angle (theta2)")
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
        
        # theta1, theta2 are now in robot convention (0°=UP, CW+)
        
        # Send angles to serial ports
        try:
            # Wait for user confirmation before sending
            print(f"\n⏸️  Ready to send data to robot:")
            print(f"   θ1 = {theta1:.1f}° → {PORT1}")
            print(f"   θ2 = {theta2:.1f}° → {PORT2}")
            
            user_input = input("\nPress Enter to SEND data, or 's' to SKIP: ").strip().lower()
            
            if user_input == 's':
                print("📋 Data transmission SKIPPED by user")
                # Still show visualization
                visualize_robot(x, y, theta1, theta2, L1, L2)
                print("-" * 60)
                continue
            
            # Send angles to serial ports
            send_angles(ser1, ser2, theta1, theta2)
            
            # Show visualization
            visualize_robot(x, y, theta1, theta2, L1, L2)
            
        except Exception as e:
            print(f"❌ Error during serial transmission: {e}")
            print(f"   Error type: {type(e).__name__}")
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