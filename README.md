# Arm Kinematics Project

This project explores the principles of forward and inverse kinematics for robotic arms, with a focus on 2D arm configurations. It provides implementations of various inverse kinematics solvers and uses Pygame for visualization to demonstrate the arm's movement and reachability.

The project has been refactored for improved modularity and organization.

## Features

*   **Forward Kinematics:** Implicitly calculated for visualization (determining joint and end-effector positions from angles).
*   **Inverse Kinematics:** Centralized solvers for 1, 2 (using fsolve and geometric methods), and 3 links.
*   **Visualization:** Pygame-based graphical simulation of arm movements.
*   **Modular Design:** Utilizes a base class for common functionality and derived classes for specific arm configurations, making it easier to add new types of arms.

## Project Structure

```
arm-kinematics/
├── README.md                   <- This file
├── src/
│   ├── __init__.py             <- Marks src as a Python package
│   ├── base_arm.py             <- Base class for arm simulations (Pygame setup, main loop)
│   ├── kinematics_solvers.py   <- Centralized inverse kinematics functions
│   ├── main.py                 <- Main entry point for running selected simulations
│   ├── arm.py                  <- ThreeLinkAnimatedArm (inherits from base_arm)
│   ├── phase.py                <- (Placeholder) Angle manipulation utilities.
│   └── systems/
│       ├── __init__.py         <- Marks systems as a Python package
│       ├── one_link_system.py              <- OneLinkMouseFollowArm (inherits from base_arm)
│       ├── two_link_system.py              <- TwoLinkMouseFollowArm (fsolve, inherits from base_arm)
│       ├── three_link_system.py            <- ThreeLinkMouseFollowArm (inherits from base_arm)
│       └── two_link_geometric_mouse_follow_arm.py <- TwoLinkGeometricMouseFollowArm (geometric, inherits from base_arm)
├── pyproject.toml              <- Project dependencies and metadata
└── uv.lock                     <- Dependency lock file (if using uv)
```
*(Note: `invkin.py` has been removed as its functionality is now in `kinematics_solvers.py`)*

## Getting Started

### Prerequisites

*   Python 3.x
*   The project dependencies (Pygame, Numpy, Scipy). These are listed in `pyproject.toml`.

### Installation

1.  Clone this repository:
    ```bash
    git clone <repository_url>
    cd arm-kinematics
    ```
2.  Install dependencies using `uv` (recommended, based on `uv.lock` and `pyproject.toml`):
    ```bash
    uv pip install .
    ```
    Alternatively, if you prefer pip:
    ```bash
    pip install -r requirements.txt # You might need to create this file manually from pyproject.toml or install directly
    pip install pygame numpy scipy
    ```

### Running the simulation

Use the `src/main.py` script and specify which arm simulation you want to run as a command-line argument. You can optionally provide link lengths using the `--links` flag.

Basic command structure:
```bash
uv run src/main.py <arm_type> [--links L1 L2 ...]
```
Replace `uv run` with `python -m` if you are not using `uv`.

Available `<arm_type>` options:

*   `animated_3link`: Runs the three-link arm simulation that animates movement to a clicked point.
*   `mouse_1link`: Runs a one-link arm whose end-effector follows the mouse cursor.
*   `mouse_2link_fsolve`: Runs a two-link arm whose end-effector follows the mouse cursor, using the fsolve numerical solver for inverse kinematics.
*   `mouse_2link_geometric`: Runs a two-link arm whose end-effector follows the mouse cursor, using a geometric inverse kinematics solver.
*   `mouse_3link`: Runs a three-link arm whose end-effector follows the mouse cursor.

Examples:

*   Run the default animated 3-link arm:
    ```bash
    uv run src/main.py animated_3link
    ```
*   Run the mouse-following 1-link arm with a link length of 200:
    ```bash
    uv run src/main.py mouse_1link --links 200
    ```
*   Run the mouse-following 2-link arm (fsolve) with link lengths 120 and 80:
    ```bash
    uv run src/main.py mouse_2link_fsolve --links 120 80
    ```
*   Run the mouse-following 3-link arm with default lengths:
    ```bash
    uv run src/main.py mouse_3link
    ```

To see the available options and arguments, you can run:
```bash
uv run src/main.py -h
```

## Usage

The `main.py` script serves as the central hub for launching different arm simulations. Each simulation (`animated_3link`, `mouse_1link`, etc.) corresponds to a specific class in `src/` or `src/systems/` that inherits from `BaseArm` and implements its own kinematics and drawing logic.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details (Create a LICENSE file if one doesn\'t exist).