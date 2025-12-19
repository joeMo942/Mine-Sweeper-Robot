% main.m - Main Entry Point for Minesweeper Robot Simulation
% Minesweeper Robot Project with MATLAB Simulink and ROS2
%
% Usage: Run this script to start the minesweeper simulation
%
% Author: Minesweeper Project
% Date: December 2025

%% Clear environment
clear; clc; close all;

fprintf('========================================\n');
fprintf('   MINESWEEPER ROBOT SIMULATION\n');
fprintf('   MATLAB + Simulink + ROS2\n');
fprintf('========================================\n\n');

%% Step 1: Setup Project
fprintf('Step 1: Setting up project...\n');
setup_project();

%% Step 2: Load Parameters
fprintf('Step 2: Loading robot parameters...\n');
run('robot_params.m');

%% Step 3: Generate Minefield
fprintf('\nStep 3: Generating minefield...\n');
field = MinefieldGenerator(sim.grid_size(1), sim.grid_size(2), sim.mine_density, sim.num_obstacles);

% Set start position (bottom-left corner)
startPos = [1, 1];
field.generateField(startPos);
field.displayStats();

%% Step 4: Initialize Robot
fprintf('Step 4: Initializing robot...\n');
startWorldPos = field.gridToWorld(startPos);
robotParams = struct();
robotParams.max_velocity = robot.max_velocity;
robotParams.max_angular_velocity = robot.max_angular_velocity;
robotParams.width = robot.width;
robotParams.length = robot.length;
robotParams.mine_detector_range = robot.mine_detector_range;
robotParams.mine_detector_accuracy = robot.mine_detector_accuracy;
robotParams.obstacle_sensor_range = robot.obstacle_sensor_range;
robotParams.obstacle_sensor_fov = robot.obstacle_sensor_fov;
robotParams.num_obstacle_rays = robot.num_obstacle_rays;
robotParams.kp_linear = ctrl.kp_linear;
robotParams.kp_angular = ctrl.kp_angular;
robotParams.goal_tolerance = ctrl.goal_tolerance;
robotParams.angle_tolerance = ctrl.angle_tolerance;

mineRobot = MinesweeperRobot(startWorldPos, 0, robotParams);
fprintf('Robot initialized at position (%.1f, %.1f)\n', startWorldPos(1), startWorldPos(2));

%% Step 5: Initialize ROS2 Interface
fprintf('\nStep 5: Initializing ROS2 interface...\n');
ros2if = ROS2Interface(false);  % false = use real ROS2, not simulation mode

%% Step 6: Initialize Path Planner
fprintf('\nStep 6: Initializing path planner...\n');
planner = PathPlanner(sim.grid_size, sim.cell_size, path.algorithm);
fprintf('Path algorithm: %s\n', path.algorithm);

%% Step 7: Initialize Visualization
fprintf('\nStep 7: Initializing visualization...\n');
vizParams = viz;  % Save viz parameters before creating Visualization object
vizObj = Visualization(field, 'FigSize', vizParams.figure_size, ...
                              'ShowPath', vizParams.show_path, ...
                              'ShowSensorRange', vizParams.show_sensor_range);
fprintf('Visualization window opened.\n');

%% Step 8: Run Simulation
fprintf('\nStep 8: Starting simulation...\n');
fprintf('Press Ctrl+C or close the figure window to stop.\n\n');

simParams = struct();
simParams.dt = sim.dt;
simParams.maxTime = sim.max_time;
simParams.vizUpdateRate = vizParams.update_rate;
simParams.logEnabled = true;

try
    results = runSimulation(mineRobot, field, ros2if, vizObj, planner, simParams);
catch ME
    if strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
        fprintf('\nSimulation stopped (figure closed).\n');
    else
        fprintf('\nSimulation error: %s\n', ME.message);
        rethrow(ME);
    end
end

%% Step 9: Cleanup
fprintf('\nStep 9: Cleaning up...\n');
ros2if.shutdown();
fprintf('\nSimulation complete. Thank you for using Minesweeper Robot!\n');
