% SETUP_PROJECT - Initialize the Minesweeper Robot Project
%
% Run this script to set up paths and verify the project is ready to run.
%
% Usage:
%   >> setup_project
%
% After running, you can start the simulation with:
%   >> main_ros2

function setup_project()
    fprintf('\n');
    fprintf('============================================\n');
    fprintf('  MINESWEEPER ROBOT ROS2 PROJECT SETUP\n');
    fprintf('============================================\n\n');
    
    % Get project root
    projectRoot = fileparts(mfilename('fullpath'));
    if isempty(projectRoot)
        projectRoot = pwd;
    end
    
    fprintf('Project Root: %s\n\n', projectRoot);
    
    % Add paths
    fprintf('Adding paths...\n');
    addpath(genpath(projectRoot));
    fprintf('  ✓ All project folders added to MATLAB path\n\n');
    
    % Check required toolboxes
    fprintf('Checking toolboxes...\n');
    
    toolboxes = {
        'Simulink', 'simulink';
        'Stateflow', 'stateflow';
        'ROS Toolbox', 'robotics';
        'Navigation Toolbox', 'navigation'
    };
    
    allAvailable = true;
    for i = 1:size(toolboxes, 1)
        name = toolboxes{i, 1};
        product = toolboxes{i, 2};
        
        if license('test', product)
            fprintf('  ✓ %s - Available\n', name);
        else
            fprintf('  ✗ %s - NOT AVAILABLE (some features may not work)\n', name);
            allAvailable = false;
        end
    end
    fprintf('\n');
    
    % Check key files exist
    fprintf('Checking project files...\n');
    
    keyFiles = {
        'scripts/main_ros2.m', 'Main simulation'
        'scripts/buildStateflowModel.m', 'Stateflow builder'
        'config/robot_params.m', 'Robot parameters'
        'scripts/OccupancyGridWorld.m', 'World class'
        'scripts/PathPlannerROS.m', 'Path planner'
        'scripts/EKFSLAM.m', 'EKF-SLAM'
    };
    
    allFiles = true;
    for i = 1:size(keyFiles, 1)
        filePath = fullfile(projectRoot, keyFiles{i, 1});
        desc = keyFiles{i, 2};
        
        if exist(filePath, 'file')
            fprintf('  ✓ %s - %s\n', keyFiles{i, 1}, desc);
        else
            fprintf('  ✗ %s - MISSING\n', keyFiles{i, 1});
            allFiles = false;
        end
    end
    fprintf('\n');
    
    % Summary
    fprintf('============================================\n');
    if allAvailable && allFiles
        fprintf('  ✓ PROJECT READY!\n');
        fprintf('============================================\n\n');
        fprintf('To run the simulation:\n');
        fprintf('  >> main_ros2\n\n');
        fprintf('To build Stateflow model:\n');
        fprintf('  >> buildStateflowModel(''minesweeper_stateflow'')\n\n');
    else
        fprintf('  ⚠ PROJECT HAS ISSUES\n');
        fprintf('============================================\n');
        fprintf('Some toolboxes or files are missing.\n');
        fprintf('The simulation may still run with limited features.\n\n');
    end
end
