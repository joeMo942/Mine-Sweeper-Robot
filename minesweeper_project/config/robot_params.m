% robot_params.m - Robot and Simulation Parameters
% Minesweeper Robot Project with MATLAB Simulink and ROS2

%% Robot Physical Parameters
robot.max_velocity = 0.5;           % Maximum linear velocity (m/s)
robot.max_angular_velocity = 1.0;   % Maximum angular velocity (rad/s)
robot.width = 0.3;                  % Robot width (m)
robot.length = 0.4;                 % Robot length (m)
robot.wheelbase = 0.25;             % Distance between wheels (m)

%% Sensor Parameters
robot.mine_detector_range = 0.5;    % Mine detector sensing range (m)
robot.mine_detector_accuracy = 0.95; % Detection accuracy (0-1)
robot.obstacle_sensor_range = 1.0;  % Obstacle sensor range (m)
robot.obstacle_sensor_fov = pi/2;   % Field of view (radians)
robot.num_obstacle_rays = 8;        % Number of obstacle sensor rays

%% Simulation Parameters
sim.dt = 0.1;                       % Time step (seconds)
sim.max_time = 300;                 % Maximum simulation time (seconds)
sim.grid_size = [10, 10];           % Grid dimensions [rows, cols]
sim.cell_size = 1.0;                % Size of each grid cell (m)
sim.mine_density = 0.15;            % Mine density (15% of cells)
sim.num_obstacles = 5;              % Number of random obstacles

%% Visualization Parameters
viz.update_rate = 10;               % Visualization update rate (Hz)
viz.show_path = true;               % Show robot path trail
viz.show_sensor_range = true;       % Show sensor detection range
viz.figure_size = [800, 800];       % Figure window size [width, height]

%% ROS2 Parameters
ros2.domain_id = 0;                 % ROS2 domain ID
ros2.node_name = 'minesweeper_matlab';
ros2.qos_depth = 10;                % QoS history depth

%% Topic Names
ros2.topics.pose = '/robot/pose';
ros2.topics.cmd_vel = '/robot/cmd_vel';
ros2.topics.mine_detector = '/sensor/mine_detector';
ros2.topics.obstacles = '/sensor/obstacles';
ros2.topics.mine_marked = '/robot/mine_marked';
ros2.topics.simulation_status = '/simulation/status';

%% Path Planning Parameters
path.algorithm = 'boustrophedon';   % 'boustrophedon' or 'spiral'
path.safety_margin = 0.3;           % Safety margin around mines (m)
path.replanning_enabled = true;     % Enable dynamic replanning

%% Controller Parameters
ctrl.kp_linear = 1.0;               % Proportional gain for linear velocity
ctrl.kp_angular = 2.0;              % Proportional gain for angular velocity
ctrl.goal_tolerance = 0.1;          % Distance tolerance to reach goal (m)
ctrl.angle_tolerance = 0.1;         % Angle tolerance (radians)

disp('Robot parameters loaded successfully.');
