import serial
import time
import math

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

# Define your serial ports and baud rate
port1 = 'COM12'  # Change to your actual port for Joint 1 (theta1)
port2 = 'COM8'   # Change to your actual port for Joint 2 (theta2)
baud_rate = 9600  # Common baud rate

# Robot arm parameters in millimeters (mm)
L1 = 100.0  # Length of first link in mm
L2 = 80.0   # Length of second link in mm

# Open the serial ports
try:
    ser1 = serial.Serial(port1, baud_rate, timeout=1)
    ser2 = serial.Serial(port2, baud_rate, timeout=1)
    print(f"Connected to {port1} and {port2}")
    print(f"Robot arm parameters: L1={L1}mm, L2={L2}mm")
    print(f"Workspace range: {abs(L1-L2):.1f}mm to {L1+L2:.1f}mm from origin")
except Exception as e:
    print(f"Error opening ports: {e}")
    exit()

print("\n=== Robot Arm Position-to-Angle Controller ===")
print("Enter target X,Y coordinates in millimeters (mm) and the system will:")
print("1. Calculate inverse kinematics")
print("2. Send joint angles to serial ports")
print("3. Port1 (COM12) = Joint 1 angle (theta1)")
print("4. Port2 (COM8) = Joint 2 angle (theta2)")
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
            # Send theta1 to port1 and theta2 to port2
            ser1.write((str(round(theta1, 1)) + '\n').encode())
            ser2.write((str(round(theta2, 1)) + '\n').encode())
            
            print(f"✅ Joint angles calculated:")
            print(f"   θ1 (Joint 1): {theta1:.1f}° → Sent to {port1}")
            print(f"   θ2 (Joint 2): {theta2:.1f}° → Sent to {port2}")
            print(f"   Robot should move to position ({x}mm, {y}mm)")
            
        except Exception as e:
            print(f"❌ Error sending to serial ports: {e}")
        
        print("-" * 60)

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    ser1.close()
    ser2.close()
    print("Serial ports closed.")