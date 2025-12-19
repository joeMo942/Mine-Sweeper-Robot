# Minesweeper Robot Project

A MATLAB/Simulink-based minesweeper robot simulation with ROS2 integration.

## Overview

This project simulates a minesweeper robot that navigates a minefield, detects mines using sensors, and marks their locations. The robot uses a systematic coverage path planning algorithm and communicates via ROS2 topics.

## Features

- **Grid-based Minefield**: Configurable grid size, mine density, and obstacles
- **Robot Simulation**: Differential drive kinematics with realistic sensor models
- **Path Planning**: Boustrophedon (lawn mower) and spiral coverage algorithms
- **ROS2 Integration**: Publishers and subscribers for robot state and commands
- **Real-time Visualization**: 2D visualization with robot, path, mines, and obstacles
- **Simulink Controller**: Programmatically generated Simulink model for robot control

## Project Structure

```
minesweeper_project/
├── scripts/
│   ├── main.m                    # Main entry point
│   ├── setup_project.m           # Project initialization
│   ├── MinefieldGenerator.m      # Minefield generation class
│   ├── MinesweeperRobot.m        # Robot class with kinematics
│   ├── ROS2Interface.m           # ROS2 communication
│   ├── PathPlanner.m             # Path planning algorithms
│   ├── Visualization.m           # Real-time 2D visualization
│   ├── runSimulation.m           # Main simulation loop
│   ├── buildSimulinkModel.m      # Simulink model generator
│   ├── minesweeperController.m   # Controller function
│   └── test_simulation.m         # Quick test script
├── config/
│   └── robot_params.m            # Robot and simulation parameters
├── models/
│   └── (Simulink models)
└── results/
    └── (Simulation logs)
```

## Requirements

- MATLAB R2021a or later
- Simulink
- ROS Toolbox (optional - simulation mode available without it)
- Stateflow (optional)

## Quick Start

1. **Open MATLAB** and navigate to the project directory

2. **Run the main script**:
   ```matlab
   cd scripts
   main
   ```

3. **Or run a quick test**:
   ```matlab
   cd scripts
   test_simulation
   ```

## Configuration

Edit `config/robot_params.m` to customize:

- **Robot parameters**: velocity limits, sensor ranges
- **Simulation parameters**: time step, grid size, mine density
- **Visualization settings**: update rate, display options
- **ROS2 settings**: topic names, QoS settings

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/robot/pose` | geometry_msgs/Pose2D | Robot position and heading |
| `/robot/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/sensor/mine_detector` | std_msgs/Bool | Mine detection status |
| `/robot/mine_marked` | geometry_msgs/Point | Marked mine location |
| `/simulation/status` | std_msgs/String | Simulation status |

## Simulink Model

To generate the Simulink controller model:

```matlab
cd scripts
buildSimulinkModel('minesweeper_controller')
```

This creates a model with:
- ROS2 input subsystem (subscribers)
- Controller subsystem (MATLAB Function block)
- ROS2 output subsystem (publishers)

## Usage Examples

### Custom Minefield

```matlab
% Create custom 15x15 field with 20% mines
field = MinefieldGenerator(15, 15, 0.20, 8);
field.generateField([1, 1]);
field.displayStats();
```

### Different Path Algorithm

```matlab
% Use spiral path instead of boustrophedon
planner = PathPlanner([10, 10], 1.0, 'spiral');
planner.planPath([0.5, 0.5], field);
```

### Adjust Robot Speed

```matlab
% Faster robot
robotParams.max_velocity = 1.0;
robotParams.max_angular_velocity = 2.0;
```

## License

This project is provided as-is for educational purposes.

## Author

Minesweeper Project - December 2025
