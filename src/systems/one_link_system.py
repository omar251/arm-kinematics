import sys
import math
import pygame
import numpy as np
from ..base_arm import BaseArm
from ..kinematics_solvers import inverse_kinematics_one_link

class OneLinkMouseFollowArm(BaseArm):
    """
    Simulates a one-link robotic arm whose end-effector follows the mouse cursor.
    Inherits from BaseArm for common Pygame functionality.
    """
    def __init__(self, length):
        """
        Initializes the OneLinkMouseFollowArm.

        Args:
            length (float): Length of the single link.
        """
        super().__init__(caption='One-Link Mouse Follow Simulation')

        # Link lengths
        self.link_length = length

        # Arm state
        self.theta1 = 0.0 # in degrees
        # End-effector position in Pygame window coordinates
        self.x1, self.y1 = self.origin_x, self.origin_y

        # Variable to store target position from mouse
        # Initialize target to the origin
        self.target_x = self.origin_x
        self.target_y = self.origin_y
        self.prev_mouse_pos = (0, 0)


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
        Updates the arm's joint angle and end-effector position based on the target.
        """
        # Convert target from window coordinates to coordinates relative to arm origin
        target_x_rel = self.target_x - self.origin_x
        target_y_rel = self.target_y - self.origin_y

        # Calculate inverse kinematics
        # inverse_kinematics_one_link expects relative x, y
        # Check if the target is reachable within a small tolerance, otherwise the IK function handles it
        self.theta1 = inverse_kinematics_one_link(self.link_length, target_x_rel, target_y_rel)

        # Update end-effector position based on the new angle (Forward Kinematics)
        theta1_rad = math.radians(self.theta1)
        self.x1 = self.link_length * math.cos(theta1_rad) + self.origin_x
        self.y1 = self.link_length * math.sin(theta1_rad) + self.origin_y


    def draw_arm(self):
        """
        Draws the one-link arm and relevant information on the screen.
        """
        # Draw link
        pygame.draw.line(self.screen, self.black, (self.origin_x, self.origin_y), (self.x1, self.y1), 5)

        # Draw joint (end-effector)
        # Base is drawn by BaseArm
        pygame.draw.circle(self.screen, self.red, (int(self.x1), int(self.y1)), 8)   # End-effector

        # Draw the target destination (optional, but helps visualize mouse target)
        # pygame.draw.circle(self.screen, self.blue, (int(self.target_x), int(self.target_y)), 8) # Target

        # Display text (coordinates relative to origin and angle)
        text_target_x = self.font.render(f"Target x: {self.target_x - self.origin_x}", True, self.black)
        text_target_y = self.font.render(f"Target y: {self.target_y - self.origin_y}", True, self.black)
        text_current_x = self.font.render(f"Current x: {self.x1 - self.origin_x:.2f}", True, self.black)
        text_current_y = self.font.render(f"Current y: {self.y1 - self.origin_y:.2f}", True, self.black)
        text_theta1 = self.font.render(f"Theta1: {self.theta1:.2f} degrees", True, self.black)

        self.screen.blit(text_target_x, (10, 100))
        self.screen.blit(text_target_y, (10, 120))
        self.screen.blit(text_current_x, (10, 140))
        self.screen.blit(text_current_y, (10, 160))
        self.screen.blit(text_theta1, (10, 10))


# Example Usage (for direct running of this specific arm)
if __name__ == "__main__":
    # Create an instance of the one-link mouse-following arm
    arm_instance = OneLinkMouseFollowArm(150) # Set your desired link length here
    # Run the simulation loop from the BaseArm class
    arm_instance.run()
