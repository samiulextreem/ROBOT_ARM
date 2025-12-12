import serial
import time
import math

# ===== ROBOT ARM CONFIGURATION =====
# Serial Port Settings
PORT1 = 'COM9'   # Port for Joint 1 (theta1)
PORT2 = 'COM8'   # Port for Joint 2 (theta2)
BAUD_RATE = 9600
TIMEOUT = 1

# Robot Arm Physical Parameters (in millimeters)
L1 = 220.0  # Length of first link in mm
L2 = 185.0  # Length of second link in mm

# Robot Direction Configuration
CLOCKWISE_POSITIVE = False

# Interpolation Settings
INTERPOLATION_STEP = 0.5   # Max angle change per step in DEGREES (smaller = smoother)
STEP_DELAY = 0.02          # Delay between each step in seconds (smaller = faster)
X_SHIFT_DISTANCE = 20.0    # Distance to move along X axis in mm
# =====================================

def inverse_kinematics(x, y, L1=L1, L2=L2, verbose=False):
    """
    Calculate joint angles for 2-DOF planar robot arm
    Returns (theta1, theta2) in robot convention, or None if unreachable
    """
    distance = math.sqrt(x*x + y*y)
    
    # Check reachability
    if distance > (L1 + L2) or distance < abs(L1 - L2):
        return None
    
    # Calculate theta2
    cos_theta2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2)
    cos_theta2 = max(-1, min(1, cos_theta2))
    
    theta2_elbow_up = math.acos(cos_theta2)
    theta2_elbow_down = -math.acos(cos_theta2)
    
    solutions = []
    
    # Elbow up
    k1_up = L1 + L2 * math.cos(theta2_elbow_up)
    k2_up = L2 * math.sin(theta2_elbow_up)
    theta1_elbow_up = math.atan2(y, x) - math.atan2(k2_up, k1_up)
    solutions.append((theta1_elbow_up, theta2_elbow_up))
    
    # Elbow down
    k1_down = L1 + L2 * math.cos(theta2_elbow_down)
    k2_down = L2 * math.sin(theta2_elbow_down)
    theta1_elbow_down = math.atan2(y, x) - math.atan2(k2_down, k1_down)
    solutions.append((theta1_elbow_down, theta2_elbow_down))
    
    # Choose solution with most positive theta1
    best_solution = max(solutions, key=lambda sol: sol[0])
    
    theta1_deg = math.degrees(best_solution[0])
    theta2_deg = math.degrees(best_solution[1])
    
    # Convert to robot convention (0°=UP)
    if CLOCKWISE_POSITIVE:
        theta1_deg = 90 - theta1_deg
        theta2_deg = -theta2_deg
    else:
        theta1_deg = theta1_deg - 90
        theta2_deg = -theta2_deg
    
    if verbose:
        print(f"   IK: ({x:.1f}, {y:.1f}) → θ1={theta1_deg:.2f}°, θ2={theta2_deg:.2f}°")
    
    return theta1_deg, theta2_deg

def send_angles(ser1, ser2, theta1, theta2):
    """Send joint angles to the robot (fast, minimal output)"""
    theta1_rounded = round(theta1, 1)
    theta2_rounded = round(theta2, 1)
    
    ser1.write((str(theta1_rounded) + '\n').encode())
    ser2.write((str(theta2_rounded) + '\n').encode())
    ser1.flush()
    ser2.flush()

def move_to(ser1, ser2, x, y, verbose=True):
    """
    Move robot to specified X, Y position (instant jump)
    Returns True if successful, False if unreachable
    """
    result = inverse_kinematics(x, y, verbose=False)
    
    if result is None:
        if verbose:
            print(f"❌ Position ({x}, {y}) is UNREACHABLE!")
        return False
    
    theta1, theta2 = result
    send_angles(ser1, ser2, theta1, theta2)
    
    if verbose:
        print(f"✓ Moved to ({x:.1f}, {y:.1f}mm) → θ1={theta1:.1f}°, θ2={theta2:.1f}°")
    
    return True

def interpolate_angles(theta1_start, theta2_start, theta1_end, theta2_end, angle_step=INTERPOLATION_STEP):
    """
    Generate intermediate angle values between start and end angles
    Returns list of (theta1, theta2) tuples
    
    Args:
        theta1_start, theta2_start: Starting angles in degrees
        theta1_end, theta2_end: Ending angles in degrees
        angle_step: Maximum angle change per step in degrees (smaller = smoother)
    """
    d_theta1 = theta1_end - theta1_start
    d_theta2 = theta2_end - theta2_start
    
    # Find the maximum angle change
    max_delta = max(abs(d_theta1), abs(d_theta2))
    
    if max_delta < angle_step:
        return [(theta1_start, theta2_start), (theta1_end, theta2_end)]
    
    # Calculate number of steps based on largest angle change
    num_steps = int(math.ceil(max_delta / angle_step))
    
    angles = []
    for i in range(num_steps + 1):
        t = i / num_steps  # Parameter from 0 to 1
        theta1 = theta1_start + t * d_theta1
        theta2 = theta2_start + t * d_theta2
        angles.append((theta1, theta2))
    
    return angles

def move_linear_interpolated(ser1, ser2, x1, y1, x2, y2, angle_step=INTERPOLATION_STEP, step_delay=STEP_DELAY, verbose=True):
    """
    Move robot smoothly from (x1,y1) to (x2,y2) by interpolating ANGLES
    
    The stepper motor angles are gradually increased/decreased step by step
    for smooth motion.
    
    Args:
        x1, y1: Start position in mm
        x2, y2: End position in mm
        angle_step: Maximum angle change per step in degrees (smaller = smoother)
        step_delay: Time delay between each step
    
    Returns True if successful, False if unreachable
    """
    # Calculate start and end angles
    result_start = inverse_kinematics(x1, y1)
    result_end = inverse_kinematics(x2, y2)
    
    if result_start is None:
        print(f"❌ Start position ({x1:.1f}, {y1:.1f}) is UNREACHABLE!")
        return False
    
    if result_end is None:
        print(f"❌ End position ({x2:.1f}, {y2:.1f}) is UNREACHABLE!")
        return False
    
    theta1_start, theta2_start = result_start
    theta1_end, theta2_end = result_end
    
    # Generate interpolated angle path
    angle_path = interpolate_angles(theta1_start, theta2_start, theta1_end, theta2_end, angle_step)
    
    if verbose:
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        print(f"📐 Moving {distance:.1f}mm in {len(angle_path)} steps")
        print(f"   θ1: {theta1_start:.1f}° → {theta1_end:.1f}° (Δ={theta1_end-theta1_start:.1f}°)")
        print(f"   θ2: {theta2_start:.1f}° → {theta2_end:.1f}° (Δ={theta2_end-theta2_start:.1f}°)")
    
    # Execute smooth angle interpolation
    for i, (theta1, theta2) in enumerate(angle_path):
        send_angles(ser1, ser2, theta1, theta2)
        
        # Progress indicator (every 20 steps or at key points)
        if verbose and (i % 20 == 0 or i == len(angle_path) - 1):
            progress = (i / (len(angle_path) - 1)) * 100 if len(angle_path) > 1 else 100
            print(f"   {progress:.0f}% - θ1={theta1:.1f}°, θ2={theta2:.1f}°")
        
        time.sleep(step_delay)
    
    if verbose:
        print(f"✓ Reached ({x2:.1f}, {y2:.1f})")
    
    return True

def move_and_shift_x(ser1, ser2, x, y, x_shift=X_SHIFT_DISTANCE):
    """
    Move to (x, y), then smoothly shift along X axis
    
    Args:
        x, y: Initial target coordinates in mm
        x_shift: Distance to move along X axis (default 10mm)
    """
    print(f"\n🎯 Starting smooth movement sequence:")
    print(f"   Target: ({x}, {y}) → ({x + x_shift}, {y}) [{x_shift}mm shift]")
    print("-" * 50)
    
    # Get current position (assume starting from home or use last known)
    # For now, jump to initial position first
    print(f"\n📍 Step 1: Moving to start position ({x}, {y})...")
    if not move_to(ser1, ser2, x, y, verbose=True):
        return False
    
    # Wait 5 seconds before X-axis movement
    print(f"\n⏳ Waiting 5 seconds before X-axis move...")
    time.sleep(5.0)
    
    # Step 2: Smooth linear movement along X axis
    new_x = x + x_shift
    print(f"\n📍 Step 2: Smooth linear move to ({new_x}, {y})...")
    if not move_linear_interpolated(ser1, ser2, x, y, new_x, y):
        print(f"⚠️  Linear move failed!")
        return False
    
    print(f"\n✅ Movement sequence complete!")
    return True

def open_serial_ports():
    """Open and return serial port connections"""
    print("Opening serial ports...")
    
    try:
        ser1 = serial.Serial(PORT1, BAUD_RATE, timeout=TIMEOUT)
        ser2 = serial.Serial(PORT2, BAUD_RATE, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"❌ Failed to open serial ports: {e}")
        return None, None
    
    time.sleep(2)  # Wait for Arduino initialization
    print(f"✓ Connected: {PORT1} (θ1), {PORT2} (θ2)")
    print(f"✓ Robot: L1={L1}mm, L2={L2}mm, Range={abs(L1-L2):.0f}-{L1+L2:.0f}mm\n")
    
    return ser1, ser2

# ===== MAIN SCRIPT =====
if __name__ == "__main__":
    print("=" * 50)
    print("   ROBOT SMOOTH LINEAR MOVEMENT")
    print(f"   Move to (X,Y) then interpolate +{X_SHIFT_DISTANCE}mm along X")
    print(f"   Angle step: {INTERPOLATION_STEP}°, Delay: {STEP_DELAY}s")
    print("=" * 50)
    
    # Open serial ports
    ser1, ser2 = open_serial_ports()
    if ser1 is None:
        exit(1)
    
    try:
        while True:
            # Get coordinates from user
            try:
                print("\n" + "=" * 50)
                x_input = input("Enter X coordinate in mm (or 'q' to quit): ").strip()
                if x_input.lower() == 'q':
                    break
                
                y_input = input("Enter Y coordinate in mm: ").strip()
                if y_input.lower() == 'q':
                    break
                
                x = float(x_input)
                y = float(y_input)
                
            except ValueError:
                print("❌ Invalid input! Please enter numeric values.")
                continue
            
            # Execute the smooth movement sequence
            move_and_shift_x(ser1, ser2, x, y, x_shift=X_SHIFT_DISTANCE)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    
    finally:
        print("\nClosing serial ports...")
        try:
            ser1.close()
            ser2.close()
            print("✓ Ports closed. Goodbye!")
        except:
            pass

