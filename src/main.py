import sys
import os
sys.path.append(os.path.abspath(os.path.join(__file__, "../..")))
from src.arm import arm

if __name__ == "__main__":
    arm_instance = arm(100, 70, 50)
    arm_instance.run()
