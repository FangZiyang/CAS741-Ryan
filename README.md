# 2D Robotic Arm Path Planning System

Developer: Ryan

Project Start Date: January 17, 2024

This project is a system for 2D robotic arm path planning, capable of generating obstacle-avoiding paths for multi-link robotic arms.

The folders and files for this project are as follows:

- planner - Core path planning algorithms and robotic arm models
- tests - Test cases
- ui - User interface components
- examples.py - Example configurations
- main.py - Main program entry point

## Installation

To install this project, follow these steps:

1. Clone the repository to your local machine
2. Create and activate a virtual environment (recommended)
3. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage

### Running the Application

To start the application, use the following command:
```bash
python main.py
```

This will open a graphical interface where you can:
- Select predefined example scenarios
- Visualize the robotic arm and obstacles
- Run the path planning algorithm
- View planning results

### Running the Test Suite

To run the test suite, use the following command:
```bash
pytest tests/ -v
```

This will execute all test cases and generate a test report:

```
============================= test session starts ==============================
platform win32 -- Python 3.9.6, pytest-8.3.5, pluggy-1.5.0 -- C:\Users\Ryan\env\Scripts\python.exe
cachedir: .pytest_cache
rootdir: E:\course\CAS741-Ryan\src
configfile: pytest.ini
plugins: emoji-0.2.0, md-0.2.0, cov-6.1.1
collected 15 items

tests/test_astar_planner.py::test_astar_basic PASSED                    [  6%]
tests/test_astar_planner.py::test_astar_with_obstacles PASSED          [ 13%]
tests/test_astar_planner.py::test_astar_no_path PASSED                 [ 20%]
tests/test_collision.py::test_detect_collision PASSED                   [ 26%]
tests/test_collision.py::test_get_occupancy_grid PASSED                [ 33%]
tests/test_collision.py::test_get_occupancy_grid_with_limits PASSED    [ 40%]
tests/test_examples.py::test_get_all_examples PASSED                   [ 46%]
tests/test_joint_limits.py::test_joint_limits_basic PASSED             [ 53%]
tests/test_joint_limits.py::test_joint_limits_validation PASSED        [ 60%]
tests/test_nlink_arm.py::test_arm_initialization PASSED                [ 66%]
tests/test_nlink_arm.py::test_arm_update_joints PASSED                 [ 73%]
tests/test_nlink_arm.py::test_arm_forward_kinematics PASSED           [ 80%]
tests/test_nlink_arm.py::test_arm_inverse_kinematics PASSED           [ 86%]
tests/test_nlink_arm.py::test_arm_collision_detection PASSED          [ 93%]
tests/test_nlink_arm.py::test_arm_path_planning PASSED                [100%]

============================== 15 passed in 5.23s ==============================
```

You can also run specific tests by specifying the test file name:
```bash
pytest tests/test_nlink_arm.py -v
```

### Using Examples

The project includes multiple predefined example scenarios that you can select from the dropdown menu in the application. Each example has different robotic arm configurations and obstacle settings.

To add new examples, you can edit the `examples.py` file and add new configurations.

## Project Structure

### Core Components

- `planner/nlink_arm.py` - Implements the multi-link robotic arm model and kinematics calculations
- `planner/collision.py` - Implements collision detection and occupancy grid generation
- `planner/astar_planner.py` - Implements the A* path planning algorithm
- `planner/joint_limits.py` - Implements joint limit functionality

### User Interface

- `ui/example_info_window.py` - Example information window
- `ui/trajectory_plot.py` - Trajectory plotting window

## Contributing

Issues and pull requests are welcome to improve this project.

## License

This project is licensed under the MIT License.
