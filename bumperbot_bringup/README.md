# Bumperbot Bringup

This package contains the top-level launch files to bring up the Bumperbot robot, including the simulation, controllers, and visualization tools.

## robot.launch.py

This launch file is the main entry point to start the Bumperbot simulation. It includes the following launch files from other packages:

-   **`gazebo.launch.py`** (from `bumperbot_description`): Launches the Gazebo simulation environment with the Bumperbot model.
-   **`controller.launch.py`** (from `bumperbot_controller`): Launches the controllers for the Bumperbot, including the noisy and simple controllers.
-   **`joystick_teleop.launch.py`** (from `bumperbot_controller`): Launches the joystick teleoperation node to control the robot with a joystick.
-   **`display.launch.py`** (from `bumperbot_description`): Launches RViz with the Bumperbot model and configuration.

### How to Launch

To launch the Bumperbot simulation, run the following command:

```bash
ros2 launch bumperbot_bringup robot.launch.py
```
