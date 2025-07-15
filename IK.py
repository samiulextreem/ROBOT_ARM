import math
import matplotlib.pyplot as plt
import numpy as np

def inverse_kinematics(x, y, L1=10.0, L2=8.0):
    """
    Calculate joint angles for 2-DOF planar robot arm
    
    Args:
        x, y: Target coordinates
        L1: Length of first link (default: 10.0)
        L2: Length of second link (default: 8.0)
    
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

def visualize_robot(x, y, L1=10.0, L2=8.0):
    """
    Visualize the robot arm reaching for target position
    """
    # Calculate IK
    result = inverse_kinematics(x, y, L1, L2)
    
    if result is None:
        print(f"Target ({x}, {y}) is unreachable!")
        return
    
    theta1_deg, theta2_deg = result
    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)
    
    # Calculate joint positions
    base = [0, 0]
    elbow = [L1 * math.cos(theta1), L1 * math.sin(theta1)]
    end_effector = [elbow[0] + L2 * math.cos(theta1 + theta2), 
                    elbow[1] + L2 * math.sin(theta1 + theta2)]
    
    # Create plot
    plt.figure(figsize=(10, 8))
    
    # Draw workspace (reachable area)
    circle_outer = plt.Circle((0, 0), L1 + L2, fill=False, color='lightblue', linestyle='--', alpha=0.5)
    circle_inner = plt.Circle((0, 0), abs(L1 - L2), fill=False, color='lightblue', linestyle='--', alpha=0.5)
    plt.gca().add_patch(circle_outer)
    plt.gca().add_patch(circle_inner)
    
    # Draw robot arm
    plt.plot([base[0], elbow[0]], [base[1], elbow[1]], 'b-', linewidth=8, label='Link 1')
    plt.plot([elbow[0], end_effector[0]], [elbow[1], end_effector[1]], 'r-', linewidth=8, label='Link 2')
    
    # Draw joints
    plt.plot(base[0], base[1], 'ko', markersize=12, label='Base')
    plt.plot(elbow[0], elbow[1], 'go', markersize=10, label='Elbow')
    plt.plot(end_effector[0], end_effector[1], 'ro', markersize=10, label='End Effector')
    
    # Draw target
    plt.plot(x, y, 'r*', markersize=15, label='Target')
    
    # Add grid and labels
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(f'Robot Arm IK Solution\nTarget: ({x}, {y})\nAngles: θ1={theta1_deg:.1f}°, θ2={theta2_deg:.1f}°')
    plt.legend()
    
    # Set axis limits
    max_reach = L1 + L2 + 2
    plt.xlim(-max_reach, max_reach)
    plt.ylim(-max_reach, max_reach)
    
    plt.tight_layout()
    plt.show()
    
    print(f"Joint angles: θ1={theta1_deg:.1f}°, θ2={theta2_deg:.1f}°")

def interactive_demo():
    """
    Interactive demo with multiple target positions
    """
    L1, L2 = 10.0, 8.0
    targets = [(15, 5), (10, 10), (0, 15), (-8, 6), (12, -8)]
    
    for i, (x, y) in enumerate(targets):
        print(f"\nDemo {i+1}: Target ({x}, {y})")
        visualize_robot(x, y, L1, L2)
        input("Press Enter for next demo...")

# Usage examples
if __name__ == "__main__":
    print("=== Robot Arm Inverse Kinematics ===")
    print("Link 1 length: 10.0 units")
    print("Link 2 length: 8.0 units")
    print(f"Workspace range: {abs(10.0-8.0):.1f} to {10.0+8.0:.1f} units from origin")
    print()
    
    while True:
        try:
            # Get user input
            x = float(input("Enter X coordinate (or 'q' to quit): "))
            y = float(input("Enter Y coordinate: "))
            
            print(f"\nCalculating for target: ({x}, {y})")
            
            # Calculate and visualize
            result = inverse_kinematics(x, y)
            if result:
                theta1, theta2 = result
                print(f"Joint angles: θ1={theta1:.1f}°, θ2={theta2:.1f}°")
                visualize_robot(x, y)
            else:
                print(f"Target ({x}, {y}) is unreachable!")
                print(f"Distance: {math.sqrt(x*x + y*y):.2f} units")
                print(f"Reachable range: {abs(10.0-8.0):.1f} to {10.0+8.0:.1f} units")
            
            print("-" * 50)
            
        except ValueError:
            print("Exiting...")
            break
        except KeyboardInterrupt:
            print("\nExiting...")
            break