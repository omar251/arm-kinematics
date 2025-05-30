# Arm Kinematics Project

This project explores the principles of forward and inverse kinematics for robotic arms, with a focus on 2D and 3D arm configurations. It includes implementations of kinematic solvers, and visualization tools to demonstrate the arm's movement and reachability.

## Features

* **Forward Kinematics:** Calculate the end-effector position given the joint angles.

* **Inverse Kinematics:** Solve for joint angles given a target end-effector position (currently using a 3-link system).

* **Visualization:** Pygame-based visualization of the arm's movements.

* **Modular Design:** The project is structured to support different arm configurations by adding different system files.

## Project Structure

arm-kinematics/
├── README.md                <- This file
├── src/
│   ├── init.py
│   ├── arm.py               <- Core arm class, with inverse kinematics and drawing logic
│   ├── invkin.py            <- (Placeholder) Inverse kinematics calculations.
│   ├── main.py              <- Main entry point for running the arm simulation
│   ├── phase.py             <- (Placeholder) Angle manipulation utilities.
│   └── systems/
│       ├── one_link_system.py   <- Example: A simple one-link system.
│       ├── three_link_system.py <- Implementation of a three-link system.
│       ├── two_link_system.py
│       └── two_link_system_unbounded.py
├── pyproject.toml
└── ...

## Getting Started

### Prerequisites

* Python 3.x

* Pygame

### Installation

1. Clone this repository:

git clone <repository_url>
cd arm-kinematics


2. Install dependencies:

pip install -r requirements.txt  # Create this file if you have dependencies.


Or, if you use `pyproject.toml`, you can use `uv`:

uv pip install .


### Running the simulation

python arm-kinematics/src/main.py


## Usage

The `main.py` script initializes and runs the arm simulation. The `arm.py` file contains the main arm class and the inverse kinematics solver. You can modify the parameters in `main.py` to experiment with different arm configurations.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details (Create a LICENSE file if one doesn't exist).

