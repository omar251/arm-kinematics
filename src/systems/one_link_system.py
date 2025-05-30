import sys
import math
import pygame

class OneLinkSystem:
    def __init__(self, length):
        pygame.init()
        self.width, self.height = 800, 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('One-Link System Animation')
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.blue = (0, 0, 255)
        self.link_length = length
        self.x1, self.y1 = 0, 0
        self.theta1 = 0
        self.font = pygame.font.SysFont(None, 24)
        self.prev_mouse_pos = (0, 0)

    def calculate_inverse_kinematics(self, x, y):
        x_diff = x - self.width // 2
        y_diff = y - self.height // 2
        self.theta1 = math.degrees(math.atan2(y_diff, x_diff))

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            mouse_x, mouse_y = pygame.mouse.get_pos()

            if (mouse_x, mouse_y) != self.prev_mouse_pos:
                self.prev_mouse_pos = (mouse_x, mouse_y)
                self.calculate_inverse_kinematics(mouse_x, mouse_y)

                theta1_rad = math.radians(self.theta1)
                self.x1 = self.link_length * math.cos(theta1_rad) + self.width // 2
                self.y1 = self.link_length * math.sin(theta1_rad) + self.height // 2

            self.screen.fill(self.white)
            pygame.draw.line(self.screen, self.black, (self.width // 2, self.height // 2), (self.x1, self.y1), 5)
            pygame.draw.circle(self.screen, self.blue, (self.width // 2, self.height // 2), 8)
            pygame.draw.circle(self.screen, self.black, (int(self.x1), int(self.y1)), 8)

            text_x = self.font.render(f"x: {mouse_x - self.width // 2}", True, self.black)
            text_y = self.font.render(f"y: {mouse_y - self.height // 2}", True, self.black)
            text_theta1 = self.font.render(f"Theta1: {(self.theta1):.2f} degrees", True, self.black)

            self.screen.blit(text_x, (10, 50))
            self.screen.blit(text_y, (10, 70))
            self.screen.blit(text_theta1, (10, 10))

            pygame.display.flip()

        pygame.quit()
        sys.exit()

# Usage
if __name__ == "__main__":
    one_link_system = OneLinkSystem(100)  # Set your desired link length here
    one_link_system.run()
