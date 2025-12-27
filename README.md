# Minesweeper Robot ROS2 Simulation

A complete MATLAB/Simulink simulation of a minesweeper robot with ROS2 integration, SLAM mapping, and optimized coverage path planning.

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023a+-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-Required-blue.svg)](https://www.mathworks.com/products/simulink.html)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)

## 🎯 Project Overview

This project simulates a robot that:
- **Explores** an unknown environment using optimized Boustrophedon coverage path
- **Detects mines** and obstacles using simulated Lidar and mine detector sensors
- **Builds a SLAM map** in real-time as it discovers obstacles
- **Replans paths** dynamically using A* algorithm when obstacles are found
- **Communicates via ROS2** topics for sensor data and control commands

## 📁 Project Structure

```
minesweeper_ros2/
├── main_ros2.m                    # ★ Main simulation entry point
├── setup_project.m                # Project setup script
├── README.md                      # This file
├── config/
│   └── robot_params.m             # All configuration parameters
├── docs/
│   ├── ARCHITECTURE.md            # Technical architecture docs
│   ├── architecture_diagram.png   # System architecture image
│   └── stateflow_diagram.png      # FSM diagram image
├── models/
│   └── minesweeper_stateflow.slx  # Simulink/Stateflow FSM model
├── scripts/
│   ├── buildStateflowModel.m      # Programmatic Stateflow builder
│   ├── core/                      # Core classes
│   │   ├── OccupancyGridWorld.m   # Occupancy grid map
│   │   ├── MinefieldGenerator.m   # Mine/obstacle generation
│   │   ├── MinesweeperRobot.m     # Robot model
│   │   └── Visualization.m        # Visualization utilities
│   ├── sensors/                   # Sensor simulation
│   │   ├── SensorSimulator.m      # Lidar & detector simulation
│   │   └── EKFSLAM.m              # Extended Kalman Filter SLAM
│   ├── planning/                  # Path planning & FSM
│   │   ├── SpanningTreeCoverage.m # ★ Optimized coverage path (Boustrophedon + A*)
│   │   ├── PathPlanner.m          # Basic path planner
│   │   ├── PathPlannerROS.m       # A* planner with ROS2
│   │   └── StateMachine.m         # Finite State Machine
│   └── ros2/                      # ROS2 integration
│       ├── ROS2Interface.m        # ROS2 communication
│       └── ROS2TopicManager.m     # Topic management
└── results/
    └── ...                        # Simulation outputs
```

## 🏗️ Architecture

The project follows a **6-step modular architecture**:

![Architecture Diagram](docs/architecture_diagram.png)


### Module Descriptions

| Step | Module | Description |
|------|--------|-------------|
| 1 | **Occupancy Grid World** | Creates 20x20 grid map with mines and obstacles |
| 2 | **Sensors & ROS2** | Lidar (360°, 3m range), Mine Detector, ROS2 topics |
| 3 | **EKF-SLAM** | Extended Kalman Filter for localization and mapping |
| 4 | **Stateflow FSM** | Finite State Machine: Explore, Detect, Mark, Replan |
| 5 | **Coverage Path Planner** | Boustrophedon + A* for optimal coverage |
| 6 | **Simulation Flow** | Main loop integrating all components |

## 🛤️ Coverage Path Algorithm

The robot uses an optimized **Boustrophedon (lawn-mower) pattern** with A* connections:

```
→→→→→→→→→→
          ↓
←←←←←←←←←←
↓
→→→→→→→→→→
```

### Why Boustrophedon + A*?

| Algorithm | Waypoints | Backtracking | Use Case |
|-----------|-----------|--------------|----------|
| Greedy Nearest | ~550 | High | Not recommended |
| Pure DFS | ~800 | Very High | Not recommended |
| **Boustrophedon + A*** | **~400** | **Minimal** | ✅ **Used in this project** |

The `SpanningTreeCoverage` class (`scripts/planning/SpanningTreeCoverage.m`) implements:
- **Boustrophedon pattern** for systematic row-by-row coverage
- **A* pathfinding** to connect non-adjacent cells (around obstacles)
- **Obstacle avoidance** built into the path generation

## 🔄 Stateflow State Machine

The robot behavior is controlled by a Stateflow FSM with 6 states:

![Stateflow Diagram](docs/stateflow_diagram.png)


## 🚀 Quick Start

### Prerequisites
- MATLAB R2023a or later
- Simulink
- Stateflow
- ROS Toolbox (optional, for external ROS2)
- Navigation Toolbox (for occupancyMap)

### Running the Simulation

1. **Open MATLAB** and navigate to the project folder:
   ```matlab
   cd('c:\path_to_project_folder\')
   ```

2. **Setup project** (first time only):
   ```matlab
   setup_project
   ```

3. **Run the main simulation**:
   ```matlab
   main_ros2
   ```

4. **Build the Stateflow model** (optional):
   ```matlab
   buildStateflowModel('minesweeper_stateflow')
   ```

## 📊 Visualization

The simulation displays a **dual-panel view**:

| Left Panel (Original World) | Right Panel (SLAM Map) |
|----------------------------|------------------------|
| Ground truth with all obstacles | Robot's discovered knowledge |
| Gray squares = All obstacles | Blue dots = Lidar point cloud |
| Red X = All mines | Green circles = Detected mines |
| Cyan trail = Robot path | Yellow line = Optimal path |

### Sensor Simulation

- **Lidar**: 360° scan, 5° resolution, 3m range
- **Mine Detector**: Only detects mines in current cell
- **Point Cloud**: Accumulates lidar hits with realistic noise

## 🔌 ROS2 Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/robot/pose` | `geometry_msgs/Pose2D` | Robot position (x, y, theta) |
| `/mine_alert` | `std_msgs/Bool` | Mine detection signal |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM occupancy grid |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/scan` | `sensor_msgs/LaserScan` | Lidar scan data |

### ROS2 Setup (Optional)

ROS2 is optional - the simulation runs in standalone mode if ROS2 is not available.

To enable ROS2:
1. Install MATLAB ROS Toolbox from Add-On Explorer
2. Configure Python in MATLAB preferences
3. Optionally install ROS2 Humble via WSL2 for external tools

## ⚙️ Configuration

Edit `config/robot_params.m` to customize:

```matlab
robot.max_velocity = 2.0;       % m/s
robot.max_angular_velocity = 3.0; % rad/s
sim.dt = 0.005;                  % Timestep
grid.rows = 20;                  % Grid size
grid.cols = 20;
mineDensity = 0.15;              % 15% mines
numObstacles = 10;               % Number of obstacles
```

## 📈 Features

- ✅ **Optimized Coverage Path** - Boustrophedon + A* (~400 waypoints vs ~550 with greedy)
- ✅ Occupancy Grid SLAM with real-time visualization
- ✅ Lidar point cloud simulation (MathWorks style)
- ✅ Dynamic A* path replanning
- ✅ Stateflow FSM for robot behavior
- ✅ ROS2 integration ready (optional)
- ✅ Dual-panel visualization (Original vs SLAM)
- ✅ Mine and obstacle avoidance
- ✅ Shortest path calculation after exploration

## 📝 License

This project is for educational purposes.

## 👤 Author

Yousef - Minesweeper Robot ROS2 Simulation Project
