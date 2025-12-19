function params = robot_params()
% ROBOT_PARAMS - Central configuration for Minesweeper Robot Simulation
%
% Usage:
%   params = robot_params();
%   % Access parameters:
%   params.robot.max_velocity
%   params.sim.grid_rows
%   params.lidar.range
%
% Returns a struct containing all simulation parameters.

%% ==================== ROBOT PARAMETERS ====================
params.robot.max_velocity = 2.0;           % Maximum linear velocity (m/s)
params.robot.max_angular_velocity = 3.0;   % Maximum angular velocity (rad/s)
params.robot.width = 0.3;                  % Robot width (m)
params.robot.length = 0.4;                 % Robot length (m)
params.robot.start_position = [0.5, 0.5];  % Starting position [x, y] in meters

%% ==================== WORLD/GRID PARAMETERS ====================
params.world.grid_rows = 20;               % Number of grid rows
params.world.grid_cols = 20;               % Number of grid columns  
params.world.cell_size = 1.0;              % Size of each grid cell (m)
params.world.mine_density = 0.1;           % Mine density (10% of cells)
params.world.num_obstacles = 10;           % Number of random obstacles

%% ==================== SIMULATION PARAMETERS ====================
params.sim.dt = 0.005;                     % Time step (seconds)
params.sim.max_time = 1000;                % Maximum simulation time (seconds)
params.sim.speed_multiplier = 5;           % Visualization speed (skip frames)

%% ==================== LIDAR PARAMETERS ====================
params.lidar.range = 1.0;                  % Lidar detection range (meters)
params.lidar.angle_resolution = 5;         % Angular resolution (degrees)
params.lidar.angles = 0:5:359;             % Scan angles (360 degree scan)
params.lidar.noise = 0.1;                  % Point cloud noise amplitude

%% ==================== MINE DETECTOR PARAMETERS ====================
params.mine_detector.range = 0.0;          % Detection range (0 = current cell only)
params.mine_detector.accuracy = 1.0;       % Detection accuracy (1.0 = perfect)

%% ==================== PATH PLANNING PARAMETERS ====================
params.planning.algorithm = 'coverage';    % 'coverage' or 'astar'
params.planning.goal_tolerance = 0.15;     % Distance to reach waypoint (m)
params.planning.safety_margin = 0.3;       % Safety margin around obstacles (m)

%% ==================== VISUALIZATION PARAMETERS ====================
params.viz.figure_size = [1400, 700];      % Figure size [width, height]
params.viz.robot_size = 20;                % Robot marker size
params.viz.trail_width = 1.5;              % Trail line width
params.viz.path_width = 3;                 % Optimal path line width
params.viz.lidar_point_size = 4;           % Lidar point cloud dot size

%% ==================== ROS2 PARAMETERS ====================
params.ros2.enabled = true;                % Enable ROS2 communication
params.ros2.domain_id = 0;                 % ROS2 domain ID
params.ros2.node_name = 'minesweeper_matlab';

% Topic names
params.ros2.topics.pose = '/robot/pose';
params.ros2.topics.cmd_vel = '/cmd_vel';
params.ros2.topics.mine_alert = '/mine_alert';
params.ros2.topics.scan = '/scan';
params.ros2.topics.map = '/map';

%% ==================== FSM/STATEFLOW PARAMETERS ====================
params.fsm.detect_delay = 0.2;             % Delay after detection (seconds)
params.fsm.mark_delay = 0.5;               % Delay for marking hazard (seconds)
params.fsm.replan_delay = 0.3;             % Delay for replanning (seconds)

end
