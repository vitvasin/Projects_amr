# Projects_amr

## Overview

**Projects_amr** is a ROS 2 package repository developed by the **SMR team, NECTEC**, specifically for training and research in Autonomous Mobile Robots (AMR). This repository was utilized during the **AMR Training** held on **9–10 April 2025** to provide hands-on experience with building, simulating, and deploying AMR systems.

The repository is designed to support advanced robotics tasks such as path planning, simulation, and control, leveraging the flexibility and modularity of ROS 2.

## Features

- **ROS 2 Integration**: Fully compatible with ROS 2 for modern robotics development.
- **AMR Training Resources**: Includes examples and exercises from the AMR Training (9–10 April 2025).
- **Simulation Ready**: Comes with configurations for running simulations in Gazebo or other simulators.
- **Path Planning and Navigation**: Implements algorithms for navigation, obstacle avoidance, and control.
- **Multi-language Support**: Combines Python and C++ modules for flexibility.
- **Automation Tools**: Shell scripts and Makefile/CMake configurations for streamlined workflows.

## Repository Structure

The repository is organized as follows:

- **ROS 2 Packages**: Contains specific ROS 2 packages for AMR components.
- **C++ and Python Nodes**: Implements core functionalities in both languages.
- **Launch Files**: Pre-configured launch files to run simulations and demos easily.
- **Training Materials**: Includes slides, example scripts, and datasets from the AMR Training (9–10 April 2025).
- **Utilities**: Helper scripts and configuration files for development and testing.

## Installation Instructions

To set up the project locally, follow these steps:

1. Clone the repository:
   ```bash
   git clone https://github.com/vitvasin/Projects_amr.git
   cd Projects_amr
   ```

2. Install ROS 2 (if not already installed):
   Follow the installation guide for your platform: [ROS 2 Installation](https://docs.ros.org/en/rolling/Installation.html).

3. Build the ROS 2 workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

4. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

5. Run ROS 2 launch files or individual nodes as required.

## Usage Guide

### Running a Simulation
To run a simulation, use the provided launch files. For example:
```bash
ros2 launch projects_amr simulation.launch.py
```

### Path Planning and Navigation
Launch the path planning demo:
```bash
ros2 launch projects_amr path_planning.launch.py
```

### Custom Development
You can add or modify ROS 2 nodes in the `src` directory and recompile using `colcon build`. For detailed examples, refer to the training materials in the `training` directory.

## Contributing

We welcome contributions to this repository! To contribute:

1. Fork the repository.
2. Create a new branch for your feature or bug fix:
   ```bash
   git checkout -b feature-name
   ```
3. Commit your changes and push the branch to your forked repository.
4. Create a pull request to the `main` branch of `vitvasin/Projects_amr`.

Please follow the ROS 2 best practices and include tests where applicable.

## License

This project is licensed under the [MIT License](LICENSE). Feel free to use, modify, and distribute this code in accordance with the license.

## Contact Information

For questions or support, feel free to reach out:
- **GitHub Issues**: [Open an issue](https://github.com/vitvasin/Projects_amr/issues)
- **SMR Team, NECTEC**: smr-team@example.com (replace with an actual email if needed)
