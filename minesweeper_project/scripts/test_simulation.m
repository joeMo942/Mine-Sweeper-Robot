% test_simulation.m - Quick Test Script
% Run this to test the minesweeper simulation with minimal configuration

%% Clear environment
clear; clc; close all;

fprintf('========================================\n');
fprintf('   MINESWEEPER QUICK TEST\n');
fprintf('========================================\n\n');

%% Add paths
thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir);
addpath(fullfile(thisDir, '..', 'config'));

%% Create a small test field
fprintf('Creating test minefield (5x5)...\n');
field = MinefieldGenerator(5, 5, 0.1, 2);
field.generateField([1, 1]);

%% Create robot
fprintf('Creating robot...\n');
startPos = field.gridToWorld([1, 1]);
robot = MinesweeperRobot(startPos, 0);

%% Create path planner
fprintf('Creating path planner...\n');
planner = PathPlanner([5, 5], 1.0, 'boustrophedon');
planner.planPath(startPos, field);

%% Create ROS2 interface (simulation mode)
fprintf('Creating ROS2 interface (simulation mode)...\n');
ros2if = ROS2Interface(true);  % Force simulation mode

%% Create visualization
fprintf('Creating visualization...\n');
viz = Visualization(field);

%% Run short simulation
fprintf('\nRunning test simulation (30 seconds max)...\n\n');

simParams.dt = 0.1;
simParams.maxTime = 30;
simParams.vizUpdateRate = 20;
simParams.logEnabled = true;

try
    results = runSimulation(robot, field, ros2if, viz, planner, simParams);
    
    fprintf('\n========================================\n');
    fprintf('Test completed successfully!\n');
    fprintf('========================================\n');
catch ME
    fprintf('\nTest error: %s\n', ME.message);
end

%% Cleanup
ros2if.shutdown();
