import sys
import math
import pygame
import numpy as np
from ..base_arm import BaseArm
from ..kinematics_solvers import inverse_kinematics_three_link_fsolve

class ThreeLinkMouseFollowArm(BaseArm):
    """
    Simulates a three-link robotic arm whose end-effector follows the mouse cursor.
    Inherits from BaseArm for common Pygame functionality.
    """
    def __init__(self, a1, a2, a3):
        """
        Initializes the ThreeLinkMouseFollowArm.

        Args:
            a1 (float): Length of the first link.
            a2 (float): Length of the second link.
            a3 (float): Length of the third link.
        """
        super().__init__(caption='Three-Link Mouse Follow Simulation')

        # Link lengths
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3

        # Arm state
        self.theta1, self.theta2, self.theta3 = 0.0, 0.0, 0.0 # in degrees
        # Joint positions in Pygame window coordinates
        # These will be updated based on current angles
        self.xy1 = pygame.math.Vector2(self.origin_x, self.origin_y)
        self.xy2 = pygame.math.Vector2(self.origin_x, self.origin_y)
        self.xy3 = pygame.math.Vector2(self.origin_x, self.origin_y) # End-effector position

        # Variable to store target position from mouse (in window coordinates)
        # Initialize target to the origin
        self.target_x = self.origin_x
        self.target_y = self.origin_y
        self.prev_mouse_pos = (0, 0)

        # Initial forward kinematics calculation to set initial positions
        self._update_joint_positions()


    def handle_input(self):
        """
        Handles user input, specifically tracking the mouse position to set the target.
        """
        # Get current mouse position
        mouse_x, mouse_y = pygame.mouse.get_pos()

        # Check if mouse position changed
        if (mouse_x, mouse_y) != self.prev_mouse_pos:
            self.prev_mouse_pos = (mouse_x, mouse_y)
            # Update the target position directly to mouse position
            self.target_x = mouse_x
            self.target_y = mouse_y


    def update_kinematics(self):
        """
        Updates the arm's joint angles and end-effector position based on the target.
        """
        # Convert target from window coordinates to coordinates relative to arm origin
        target_x_rel = self.target_x - self.origin_x
        target_y_rel = self.target_y - self.origin_y

        # Calculate inverse kinematics
        # inverse_kinematics_three_link_fsolve expects relative x, y
        angles = inverse_kinematics_three_link_fsolve(self.a1, self.a2, self.a3, target_x_rel, target_y_rel)

        if angles is not None: # Check if solver returned valid angles
            self.theta1, self.theta2, self.theta3 = angles

            # Update joint positions based on the new angles (Forward Kinematics)
            self._update_joint_positions()
        else:
            # If IK fails, the arm stays in its last valid position.
            # Warning is printed by the inverse_kinematics_three_link_fsolve function.
            pass


    def _update_joint_positions(self):
        """
        Calculates the Pygame window coordinates of the joints based on current angles.
        (Forward Kinematics)
        """
        theta1_rad = np.radians(self.theta1)
        theta2_rad = np.radians(self.theta2)
        theta3_rad = np.radians(self.theta3)

        # Calculate position of joint 1 (end of link 1) relative to origin
        j1_x_rel = self.a1 * np.cos(theta1_rad)
        j1_y_rel = self.a1 * np.sin(theta1_rad)

        # Calculate position of joint 2 (end of link 2) relative to joint 1
        j2_x_rel = j1_x_rel + self.a2 * np.cos(theta1_rad + theta2_rad)
        j2_y_rel = j1_y_rel + self.a2 * np.sin(theta1_rad + theta2_rad)

        # Calculate position of end-effector (end of link 3) relative to joint 2
        ee_x_rel = j2_x_rel + self.a3 * np.cos(theta1_rad + theta2_rad + theta3_rad)
        ee_y_rel = j2_y_rel + self.a3 * np.sin(theta1_rad + theta2_rad + theta3_rad)

        # Convert relative positions to Pygame window coordinates
        self.xy1[0] = j1_x_rel + self.origin_x
        self.xy1[1] = j1_y_rel + self.origin_y

        self.xy2[0] = j2_x_rel + self.origin_x
        self.xy2[1] = j2_y_rel + self.origin_y

        self.xy3[0] = ee_x_rel + self.origin_x
        self.xy3[1] = ee_y_rel + self.origin_y


    def draw_arm(self):
        """
        Draws the three-link arm and relevant information on the screen.
        """
        # Draw links
        pygame.draw.line(self.screen, self.black, (self.origin_x, self.origin_y), (self.xy1[0], self.xy1[1]), 5)
        pygame.draw.line(self.screen, self.black, (self.xy1[0], self.xy1[1]), (self.xy2[0], self.xy2[1]), 5)
        pygame.draw.line(self.screen, self.black, (self.xy2[0], self.xy2[1]), (self.xy3[0], self.xy3[1]), 5)

        # Draw joints and end-effector
        # Base is drawn by BaseArm
        pygame.draw.circle(self.screen, self.black, (int(self.xy1[0]), int(self.xy1[1])), 8) # Joint 1
        pygame.draw.circle(self.screen, self.red, (int(self.xy2[0]), int(self.xy2[1])), 8)   # Joint 2
        pygame.draw.circle(self.screen, self.red, (int(self.xy3[0]), int(self.xy3[1])), 8)   # End-effector

        # Draw the target destination (optional, but helps visualize mouse target)
        pygame.draw.circle(self.screen, self.blue, (int(self.target_x), int(self.target_y)), 8) # Target


        # Display text (coordinates relative to origin and angles)
        text_target_x = self.font.render(f"Target x: {self.target_x - self.origin_x}", True, self.black)
        text_target_y = self.font.render(f"Target y: {self.target_y - self.origin_y}", True, self.black)
        text_current_x = self.font.render(f"Current x: {self.xy3[0] - self.origin_x:.2f}", True, self.black)
        text_current_y = self.font.render(f"Current y: {self.xy3[1] - self.origin_y:.2f}", True, self.black)

        text_theta1 = self.font.render(f"Theta1: {self.theta1:.2f} degrees", True, self.black)
        text_theta2 = self.font.render(f"Theta2: {self.theta2:.2f} degrees", True, self.black)
        text_theta3 = self.font.render(f"Theta3: {self.theta3:.2f} degrees", True, self.black)


        self.screen.blit(text_target_x, (10, 100))
        self.screen.blit(text_target_y, (10, 120))
        self.screen.blit(text_current_x, (10, 140))
        self.screen.blit(text_current_y, (10, 160))
        self.screen.blit(text_theta1, (10, 10))
        self.screen.blit(text_theta2, (10, 30))
        self.screen.blit(text_theta3, (10, 50))


# Example Usage (for direct running of this specific arm)
if __name__ == "__main__":
    # Create an instance of the three-link mouse-following arm
    arm_instance = ThreeLinkMouseFollowArm(100, 70, 50) # Set your desired link lengths here
    # Run the simulation loop from the BaseArm class
    arm_instance.run()
