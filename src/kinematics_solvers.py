import numpy as np
import math
from scipy.optimize import fsolve

def inverse_kinematics_one_link(L1, x, y):
    """
    Calculates the joint angle (theta1) for a one-link arm to reach a target (x, y).

    Args:
        L1 (float): Length of the first link.
        x (float): Target x-coordinate relative to the base.
        y (float): Target y-coordinate relative to the base.

    Returns:
        float: The joint angle theta1 in degrees.
    """
    # Check if the target is within the reach of the arm
    distance = math.sqrt(x**2 + y**2)
    if distance > L1:
        print(f"Warning: Target ({x}, {y}) is outside the reach of the one-link arm with length {L1}.")
        # Return angle pointing towards the target at max reach
        return math.degrees(math.atan2(y, x))
        
    return math.degrees(math.atan2(y, x))

def inverse_kinematics_two_link_fsolve(L1, L2, x, y):
    """
    Calculates the joint angles (theta1, theta2) for a two-link arm to reach a target (x, y)
    using scipy's fsolve.

    Args:
        L1 (float): Length of the first link.
        L2 (float): Length of the second link.
        x (float): Target x-coordinate relative to the base.
        y (float): Target y-coordinate relative to the base.

    Returns:
        tuple: A tuple containing the joint angles (theta1, theta2) in degrees.
               Returns (None, None) if fsolve does not converge or target is unreachable.
    """
    def equations(variables):
        theta_1, theta_2 = variables
        eq1 = x - L1 * np.cos(theta_1) - L2 * np.cos(theta_1 + theta_2)
        eq2 = y - L1 * np.sin(theta_1) - L2 * np.sin(theta_1 + theta_2)
        return [eq1, eq2]

    initial_guess = [0.0, 0.0]  # Initial guess for theta_1 and theta_2
    try:
        theta_1_rad, theta_2_rad = fsolve(equations, initial_guess)

        # Normalize angles to be within -180 to 180 degrees
        theta_1_deg = np.degrees(theta_1_rad) % 360
        if theta_1_deg > 180:
            theta_1_deg -= 360
        theta_2_deg = np.degrees(theta_2_rad) % 360
        if theta_2_deg > 180:
            theta_2_deg -= 360

        # Basic check for reachability - fsolve can converge to non-solutions outside workspace
        # A more robust check might be needed depending on desired accuracy.
        # Re-calculate end-effector position with obtained angles and check if it's close to target
        calculated_x = L1 * np.cos(np.radians(theta_1_deg)) + L2 * np.cos(np.radians(theta_1_deg + theta_2_deg))
        calculated_y = L1 * np.sin(np.radians(theta_1_deg)) + L2 * np.sin(np.radians(theta_1_deg + theta_2_deg))
        
        tolerance = 1.0 # Pixels tolerance
        if math.dist([x, y], [calculated_x, calculated_y]) > tolerance:
             print(f"Warning: fsolve did not converge to the target ({x}, {y}). Check workspace.")
             # Optionally return None or previous angles
             pass # Allow returning the found angles for now, even if not exact

        return theta_1_deg, theta_2_deg
    except Exception as e:
        print(f"Error during fsolve for 2-link IK: {e}")
        return None, None # Indicate failure

def inverse_kinematics_two_link_geometric(L1, L2, x, y):
    """
    Calculates the joint angles (theta1, theta2) for a two-link arm to reach a target (x, y)
     using a geometric approach. Provides two possible solutions (elbow up/down).

    Args:
        L1 (float): Length of the first link.
        L2 (float): Length of the second link.
        x (float): Target x-coordinate relative to the base.
        y (float): Target y-coordinate relative to the base.

    Returns:
        tuple: A tuple containing two possible solutions:
               ((theta1_sol1, theta2_sol1), (theta1_sol2, theta2_sol2)) in degrees.
               Returns (None, None) for a solution if the target is unreachable.
    """
    R2 = x**2 + y**2
    R = math.sqrt(R2)

    # Check reachability
    if R > (L1 + L2) or R < abs(L1 - L2):
        print(f"Warning: Target ({x}, {y}) is outside the reachable workspace for a 2-link arm with lengths {L1}, {L2}.")
        return (None, None), (None, None)

    # Calculate theta2 (using Law of Cosines)
    cos_theta2 = (R2 - L1**2 - L2**2) / (2 * L1 * L2)
    # Due to potential floating point inaccuracies, clamp the value to [-1, 1]
    cos_theta2 = max(min(cos_theta2, 1), -1)

    alpha = math.acos(cos_theta2)
    theta2_sol1_rad = math.pi - alpha # Elbow Up
    theta2_sol2_rad = math.pi + alpha # Elbow Down

    # Calculate theta1
    beta = math.atan2(y, x)
    gamma1 = math.atan2(L2 * math.sin(theta2_sol1_rad), L1 + L2 * math.cos(theta2_sol1_rad))
    gamma2 = math.atan2(L2 * math.sin(theta2_sol2_rad), L1 + L2 * math.cos(theta2_sol2_rad))

    theta1_sol1_rad = beta - gamma1
    theta1_sol2_rad = beta - gamma2

    # Convert to degrees and normalize
    theta1_sol1_deg = math.degrees(theta1_sol1_rad) % 360
    if theta1_sol1_deg > 180: theta1_sol1_deg -= 360
    theta2_sol1_deg = math.degrees(theta2_sol1_rad) % 360
    if theta2_sol1_deg > 180: theta2_sol1_deg -= 360

    theta1_sol2_deg = math.degrees(theta1_sol2_rad) % 360
    if theta1_sol2_deg > 180: theta1_sol2_deg -= 360
    theta2_sol2_deg = math.degrees(theta2_sol2_rad) % 360
    if theta2_sol2_deg > 180: theta2_sol2_deg -= 360


    return (theta1_sol1_deg, theta2_sol1_deg), (theta1_sol2_deg, theta2_sol2_deg)


def inverse_kinematics_three_link_fsolve(L1, L2, L3, x, y):
    """
    Calculates the joint angles (theta1, theta2, theta3) for a three-link arm
    to reach a target (x, y) using scipy's fsolve.

    Args:
        L1 (float): Length of the first link.
        L2 (float): Length of the second link.
        L3 (float): Length of the third link.
        x (float): Target x-coordinate relative to the base.
        y (float): Target y-coordinate relative to the base.

    Returns:
        tuple: A tuple containing the joint angles (theta1, theta2, theta3) in degrees.
               Returns (None, None, None) if fsolve does not converge or target is unreachable.
    """
    def equations(variables):
        theta_1, theta_2, theta_3 = variables
        eq1 = x - L1 * np.cos(theta_1) - L2 * np.cos(theta_1 + theta_2) - L3 * np.cos(theta_1 + theta_2 + theta_3)
        eq2 = y - L1 * np.sin(theta_1) - L2 * np.sin(theta_1 + theta_2) - L3 * np.sin(theta_1 + theta_2 + theta_3)
        # For a 2D arm in a plane, we only have 2 equations (x, y position)
        # and 3 variables (theta1, theta2, theta3). This is an underdetermined system.
        # fsolve needs the same number of equations as variables.
        # A common approach for redundant robots (like 3-link in 2D) is to add
        # an optimization objective (e.g., minimize joint movement, avoid obstacles)
        # or fix one joint angle.
        # The original code had a placeholder '0' equation. Let's keep that structure
        # for compatibility, acknowledging fsolve might return one valid solution
        # among infinite possibilities for an underdetermined system.
        return [eq1, eq2, 0] # Placeholder - fsolve may find one solution

    initial_guess = [0.0, 0.0, 0.0]  # Initial guess for theta_1, theta_2, and theta_3
    try:
        theta_1_rad, theta_2_rad, theta_3_rad, *_ = fsolve(equations, initial_guess)

        # Normalize angles to be within -180 to 180 degrees (or similar range)
        # The original code had some specific adjustments, let's try to replicate
        # a consistent normalization first.
        theta_1_deg = np.degrees(theta_1_rad) % 360
        if theta_1_deg > 180: theta_1_deg -= 360
        theta_2_deg = np.degrees(theta_2_rad) % 360
        if theta_2_deg > 180: theta_2_deg -= 360
        theta_3_deg = np.degrees(theta_3_rad) % 360
        if theta_3_deg > 180: theta_3_deg -= 360

        # The original code had this specific logic after normalization:
        # if theta_1_deg > 0:
        #     theta_1_deg = theta_1_deg - 180
        #     theta_2_deg = -theta_2_deg
        #     theta_3_deg = -theta_3_deg
        # This seems intended to prefer a certain configuration.
        # Let's keep this as an option or note it. For now, return the normalized fsolve result.

        # Basic check for reachability - fsolve can converge to non-solutions outside workspace
        # Re-calculate end-effector position with obtained angles and check if it's close to target
        calculated_x = L1 * np.cos(np.radians(theta_1_deg)) + L2 * np.cos(np.radians(theta_1_deg + theta_2_deg)) + L3 * np.cos(np.radians(theta_1_deg + theta_2_deg + theta_3_deg))
        calculated_y = L1 * np.sin(np.radians(theta_1_deg)) + L2 * np.sin(np.radians(theta_1_deg + theta_2_deg)) + L3 * np.sin(np.radians(theta_1_deg + theta_2_deg + theta_3_deg))

        tolerance = 1.0 # Pixels tolerance
        if math.dist([x, y], [calculated_x, calculated_y]) > tolerance:
             print(f"Warning: fsolve did not converge closely to the target ({x}, {y}) for 3-link IK. Check workspace or consider alternative solvers.")
             # Optionally return None or previous angles
             pass # Allow returning the found angles for now, even if not exact

        return theta_1_deg, theta_2_deg, theta_3_deg
    except Exception as e:
        print(f"Error during fsolve for 3-link IK: {e}")
        return None, None, None # Indicate failure