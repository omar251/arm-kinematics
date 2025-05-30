import sys
import os
import argparse

# Add the project root directory to the Python path
project_root = os.path.abspath(os.path.join(__file__, "..", ".."))
sys.path.insert(0, project_root)

from src.arm import ThreeLinkAnimatedArm
from src.systems.one_link_system import OneLinkMouseFollowArm
from src.systems.two_link_system import TwoLinkMouseFollowArm
from src.systems.two_link_geometric_mouse_follow_arm import TwoLinkGeometricMouseFollowArm
from src.systems.three_link_system import ThreeLinkMouseFollowArm

def main():
    parser = argparse.ArgumentParser(description="Run various robotic arm kinematics simulations.")
    parser.add_argument("arm_type", help="Type of arm simulation to run. Choose from: animated_3link, mouse_1link, mouse_2link_fsolve, mouse_2link_geometric, mouse_3link")
    parser.add_argument("--links", nargs="+", type=float, default=[], help="Space-separated list of link lengths (e.g., --links 100 50 30)")
    args = parser.parse_args()
    arm_instance = None
    link_lengths = args.links

    if args.arm_type == "animated_3link":
        if not link_lengths:
            link_lengths = [100, 70, 50]
        if len(link_lengths) == 3:
            arm_instance = ThreeLinkAnimatedArm(*link_lengths)
        else:
            print("Error: 'animated_3link' simulation requires exactly 3 link lengths.")
            return

    elif args.arm_type == "mouse_1link":
        if not link_lengths:
            link_lengths = [150]
        if len(link_lengths) == 1:
            arm_instance = OneLinkMouseFollowArm(*link_lengths)
        else:
            print("Error: 'mouse_1link' simulation requires exactly 1 link length.")
            return

    elif args.arm_type == "mouse_2link_fsolve":
        if not link_lengths:
            link_lengths = [100, 100]
        if len(link_lengths) == 2:
            arm_instance = TwoLinkMouseFollowArm(*link_lengths, solver='fsolve')
        else:
            print("Error: 'mouse_2link_fsolve' simulation requires exactly 2 link lengths.")
            return

    elif args.arm_type == "mouse_2link_geometric":
        if not link_lengths:
            link_lengths = [100, 100]
        if len(link_lengths) == 2:
            arm_instance = TwoLinkGeometricMouseFollowArm(*link_lengths)
        else:
            print("Error: 'mouse_2link_geometric' simulation requires exactly 2 link lengths.")
            return

    elif args.arm_type == "mouse_3link":
        if not link_lengths:
            link_lengths = [100, 70, 50]
        if len(link_lengths) == 3:
            arm_instance = ThreeLinkMouseFollowArm(*link_lengths)
        else:
            print("Error: 'mouse_3link' simulation requires exactly 3 link lengths.")
            return

    else:
        print(f"Error: Unknown arm type '{args.arm_type}'.")
        print("Available types: animated_3link, mouse_1link, mouse_2link_fsolve, mouse_2link_geometric, mouse_3link")
        return

    if arm_instance:
        print(f"Running {args.arm_type} simulation...")
        arm_instance.run()

if __name__ == "__main__":
    main()

