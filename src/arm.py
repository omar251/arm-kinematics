import sys
import math
import pygame
import numpy as np
import time
from .base_arm import BaseArm
from .kinematics_solvers import inverse_kinematics_three_link_fsolve

class ThreeLinkAnimatedArm(BaseArm):
    """
    Simulates a three-link robotic arm with animated movement to a clicked target.
    Inherits from BaseArm for common Pygame functionality.
    """
    def __init__(self, a1, a2, a3):
        """
        Initializes the ThreeLinkAnimatedArm.

        Args:
            a1 (float): Length of the first link.
            a2 (float): Length of the second link.
            a3 (float): Length of the third link.
        """
        super().__init__(caption='Three-Link Animated Simulation')

        # Link lengths
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3

        # Initialize angles (in degrees)
        self.theta1, self.theta2, self.theta3 = 0.0, 0.0, 0.0

        # Initialize joint positions (relative to Pygame window coordinates)
        # These will be updated based on current angles
        self.xy1 = pygame.math.Vector2(self.origin_x, self.origin_y)
        self.xy2 = pygame.math.Vector2(self.origin_x, self.origin_y)
        self.xy3 = pygame.math.Vector2(self.origin_x, self.origin_y) # End-effector position

        # Variables for animated movement
        # Start at the origin initially
        self.source = pygame.math.Vector2(self.origin_x, self.origin_y)
        # Destination starts at the origin
        self.destination = pygame.math.Vector2(self.origin_x, self.origin_y)
        self.points = [] # List of intermediate points for animation
        self._animating = False

        # Initial forward kinematics calculation to set initial positions
        self._update_joint_positions()

    def handle_input(self):
        """
        Handles user input, specifically mouse clicks to set a new target destination.
        """
        mouse_x, mouse_y = pygame.mouse.get_pos()
        # Check if left mouse button is pressed
        if pygame.mouse.get_pressed()[0]:
            # Restrict movement to upper half of the screen if needed (optional, based on original code)
            # if mouse_x < self.origin_x:
            #     mouse_x = self.origin_x
            # if mouse_y > self.origin_y:
            #     mouse_y = self.origin_y

            new_destination = pygame.math.Vector2(mouse_x, mouse_y)

            # Only update if the destination has significantly changed to avoid unnecessary calculation
            if (new_destination - self.destination).length() > 5: # Use a small tolerance
                self.destination = new_destination
                # Start path from current end-effector, but convert to Vector2 explicitly
                self.source = pygame.math.Vector2(self.xy3[0], self.xy3[1])
                self.calculate_path_points()
                self._animating = True

    def update_kinematics(self):
        """
        Updates the arm\'s joint angles and positions, progressing along the animation path.
        """
        if self._animating and len(self.points) > 0:
            # Get the next point from the path
            target_point_window = self.points.pop(0)

            # Convert target point to coordinates relative to the arm\'s origin (center of screen)
            target_x_rel = target_point_window[0] - self.origin_x
            target_y_rel = target_point_window[1] - self.origin_y

            # Calculate inverse kinematics for the target point
            angles = inverse_kinematics_three_link_fsolve(self.a1, self.a2, self.a3, target_x_rel, target_y_rel)

            if angles is not None:
                self.theta1, self.theta2, self.theta3 = angles
                # Update joint positions based on the new angles (Forward Kinematics)
                self._update_joint_positions()
            else:
                # If IK fails, stop animating or handle appropriately
                print(f"IK failed for target ({target_x_rel}, {target_y_rel}), stopping animation.")
                self.points = []
                self._animating = False
                # Do not update positions if angles are invalid - arm stays in place

            # Add a small delay for animation effect (optional, based on original code)
            # time.sleep(0.005) # Use clock.tick() in base class for frame rate control instead

        elif self._animating and len(self.points) == 0:
            # Animation is complete
            self._animating = False

    def _update_joint_positions(self):
        """
        Calculates the Pygame window coordinates of the joints based on current angles.
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
        pygame.draw.circle(self.screen, self.black, (int(self.xy1[0]), int(self.xy1[1])), 8) # Joint 1
        pygame.draw.circle(self.screen, self.red, (int(self.xy2[0]), int(self.xy2[1])), 8)   # Joint 2
        pygame.draw.circle(self.screen, self.red, (int(self.xy3[0]), int(self.xy3[1])), 8)   # End-effector

        # Draw the target destination
        pygame.draw.circle(self.screen, self.blue, (int(self.destination[0]), int(self.destination[1])), 8)

        # Display text (coordinates relative to origin and angles)
        text_target_x = self.font.render(f"Target x: {self.destination[0] - self.origin_x}", True, self.black)
        text_target_y = self.font.render(f"Target y: {self.destination[1] - self.origin_y}", True, self.black)
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


    def calculate_path_points(self):
        """
        Generates intermediate points for animation from the current end-effector
        position to the target destination.
        """
        # Calculate distance in window coordinates
        distance = int(math.hypot(self.destination[0] - self.source[0], self.destination[1] - self.source[1]))

        if distance > 0:
            # Determine number of points (can be adjusted for smoother/faster animation)
            num_points = max(50, distance // 5) # Minimum 50 points, or distance/5 for longer paths

            # Calculate spacing between points
            spacing = 1.0 # Move 1 pixel at a time

            # Calculate the number of steps based on spacing
            num_steps = int(distance / spacing)
            if num_steps == 0: # Handle cases where distance is less than spacing
                 num_steps = 1
                 spacing = distance


            self.points = [self.source] # Start with the source point
            # Generate intermediate points
            for i in range(1, num_steps):
                pt_x = self.source[0] + i * spacing * (self.destination[0] - self.source[0]) / distance
                pt_y = self.source[1] + i * spacing * (self.destination[1] - self.source[1]) / distance
                self.points.append(pygame.math.Vector2(pt_x, pt_y))

            # Ensure the final destination is included if not already the last point
            if not self.points or self.points[-1] != self.destination:
                 self.points.append(self.destination)

        else:
            self.points = [] # No points needed if distance is 0

# Example Usage (for direct running of this specific arm)
if __name__ == "__main__":
    # Create an instance of the three-link animated arm
    arm_instance = ThreeLinkAnimatedArm(100, 70, 50)
    # Run the simulation loop from the BaseArm class
    arm_instance.run()
