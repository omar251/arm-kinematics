import numpy as np  
from math import *

# Define joint limits
lower_limits = np.array([-pi/2, -pi, -pi])  
upper_limits = np.array([pi/2, pi, pi])

def calculate_ik(target_position):
    # Sample inverse kinematics solution
    joint_angles = np.array([-1.5, 1.5, -1.2])  

    # Enforce joint constraints
    joint_angles = np.maximum(joint_angles, lower_limits)  
    joint_angles = np.minimum(joint_angles, upper_limits)
    
    # If out of bounds, set safe position
    if (np.any(joint_angles == lower_limits) or 
        np.any(joint_angles == upper_limits)):
        
        print("Joint limit exceeded. Moving to safe position.")
        joint_angles = np.array([0, 0, 0])

    return joint_angles

target_pos = np.array([2, 1, 0.5])  
joint_angles = calculate_ik(target_pos)

print(joint_angles)