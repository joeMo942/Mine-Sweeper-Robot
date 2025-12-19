# Minesweeper Robot - System Architecture

## High-Level Architecture

```mermaid
graph TB
    subgraph "Physical Layer"
        WORLD[Occupancy Grid World<br/>20x20 Grid + Mines + Obstacles]
        ROBOT[Robot Platform<br/>Position, Heading, Velocity]
    end
    
    subgraph "Sensor Layer"
        LIDAR[Lidar Sensor<br/>360° Scan, 3m Range]
        MINE_DET[Mine Detector<br/>Current Cell Only]
        ODOM[Odometry<br/>Pose Tracking]
    end
    
    subgraph "Perception Layer"
        SLAM[EKF-SLAM<br/>State Estimation + Mapping]
        DISCOVERY[Obstacle Discovery<br/>Known vs Unknown]
    end
    
    subgraph "Decision Layer"
        FSM[Stateflow FSM<br/>6 States]
        PLANNER[A* Path Planner<br/>Dynamic Replanning]
    end
    
    subgraph "Communication Layer"
        ROS2[ROS2 Topics<br/>Publishers & Subscribers]
    end
    
    WORLD --> LIDAR
    WORLD --> MINE_DET
    ROBOT --> ODOM
    
    LIDAR --> SLAM
    ODOM --> SLAM
    MINE_DET --> FSM
    
    SLAM --> DISCOVERY
    DISCOVERY --> PLANNER
    DISCOVERY --> FSM
    
    FSM --> PLANNER
    PLANNER --> ROBOT
    
    SLAM --> ROS2
    FSM --> ROS2
    PLANNER --> ROS2
```

## State Machine Flow

```mermaid
stateDiagram-v2
    [*] --> Explore
    
    Explore --> DetectObstacle: obstacle_detected
    Explore --> DetectMine: mine_alert
    Explore --> MissionComplete: path_complete
    
    DetectObstacle --> UpdateSLAM: after(0.2s)
    DetectMine --> UpdateSLAM: after(0.5s)
    
    UpdateSLAM --> ReplanPath: after(0.3s)
    ReplanPath --> Explore: after(0.5s)
    
    MissionComplete --> [*]
```

## Data Flow

```mermaid
sequenceDiagram
    participant World as Occupancy Grid
    participant Robot as Robot
    participant Lidar as Lidar Sensor
    participant SLAM as EKF-SLAM
    participant FSM as State Machine
    participant Planner as Path Planner
    
    Robot->>Lidar: Request Scan
    Lidar->>World: Ray Casting
    World-->>Lidar: Hit Points
    Lidar->>SLAM: Point Cloud
    SLAM->>FSM: obstacle_detected
    FSM->>Planner: replan_flag
    Planner->>Robot: cmd_vel
```

## Component Details

### Step 1: Occupancy Grid World
- **File**: `OccupancyGridWorld.m`
- **Function**: Creates and manages the simulation environment
- **Features**:
  - Configurable grid size (default 20x20)
  - Random mine placement (15% density)
  - Random obstacle placement (10 obstacles)
  - Coordinate conversion (grid ↔ world)

### Step 2: Sensors & ROS2
- **Files**: `SensorSimulator.m`, `ROS2TopicManager.m`
- **Lidar**: 360° scan, 5° resolution, 3m range
- **Mine Detector**: Binary detection on current cell
- **Topics**: `/scan`, `/odom`, `/mine_alert`, `/map`, `/cmd_vel`

### Step 3: EKF-SLAM
- **File**: `EKFSLAM.m`
- **Function**: Extended Kalman Filter for simultaneous localization and mapping
- **Inputs**: Lidar scans, odometry
- **Outputs**: Robot pose estimate, occupancy map

### Step 4: Stateflow FSM
- **Files**: `StateMachine.m`, `buildStateflowModel.m`
- **States**: 
  1. Explore - Following coverage path
  2. DetectObstacle - Obstacle discovered
  3. UpdateSLAM - Add to known map
  4. ReplanPath - A* recalculation
  5. DetectMine - Mine detected
  6. MissionComplete - All cells explored

### Step 5: A* Path Planner
- **File**: `PathPlannerROS.m`
- **Algorithm**: A* with Manhattan heuristic
- **Features**:
  - Coverage path generation
  - Dynamic replanning on discovery
  - Obstacle and mine avoidance

### Step 6: Simulation Flow
- **File**: `main_ros2.m`
- **Function**: Main entry point integrating all components
- **Features**:
  - Dual-panel visualization
  - Real-time lidar point cloud
  - Dynamic path updates
