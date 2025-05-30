import sys
import math
import pygame
from abc import ABC, abstractmethod

class BaseArm(ABC):
    """
    Base class for robotic arm simulations using Pygame.
    Handles Pygame initialization, main loop, event handling, and basic drawing.
    Subclasses must implement specific arm kinematics and drawing logic.
    """
    def __init__(self, width=800, height=600, caption='Robotic Arm Simulation'):
        """
        Initializes the Pygame environment and screen.

        Args:
            width (int): Width of the display window.
            height (int): Height of the display window.
            caption (str): Caption for the display window.
        """
        pygame.init()
        self.width, self.height = width, height
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption(caption)
        self.clock = pygame.time.Clock()

        # Colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.blue = (0, 0, 255)

        # Origin of the arm (center of the screen)
        self.origin_x = self.width // 2
        self.origin_y = self.height // 2

        # Font setup
        self.font = pygame.font.SysFont(None, 24)

    @abstractmethod
    def handle_input(self):
        """
        Abstract method to handle user input (e.g., mouse clicks, key presses)
        to set target positions or modify joint angles.
        """
        pass

    @abstractmethod
    def update_kinematics(self):
        """
        Abstract method to calculate and update the arm's joint angles
        and end-effector position based on the current state or target.
        """
        pass

    @abstractmethod
    def draw_arm(self):
        """
        Abstract method to draw the specific arm configuration on the screen.
        """
        pass

    def draw_base(self):
        """
        Draws the arm base point (origin) on the screen.
        """
        pygame.draw.circle(self.screen, self.blue, (self.origin_x, self.origin_y), 8)

    def draw_background(self):
        """
        Fills the screen with the background color.
        """
        self.screen.fill(self.white)

    def run(self):
        """
        Main simulation loop.
        """
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            self.handle_input()
            self.update_kinematics()

            self.draw_background()
            self.draw_base()
            self.draw_arm()

            pygame.display.flip()
            self.clock.tick(60) # Cap the frame rate

        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    # This block is just for testing the base class structure
    # You cannot instantiate an abstract class directly.
    # This part won't run when imported, only if run directly.
    print("BaseArm class created. It is intended to be inherited by specific arm implementations.")
    # Example of how a derived class might look (conceptual)
    # class MyArm(BaseArm):
    #     def __init__(self):
    #         super().__init__(caption="My Arm Simulation")
    #         # Specific arm setup
    #     def handle_input(self):
    #         pass # Implement input handling
    #     def update_kinematics(self):
    #         pass # Implement kinematics
    #     def draw_arm(self):
    #         pass # Implement drawing
    # MyArm().run()
