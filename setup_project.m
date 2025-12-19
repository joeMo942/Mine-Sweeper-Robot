% SETUP_PROJECT - Initialize the Minesweeper Robot Project
%
% This script sets up the MATLAB path and verifies all required
% toolboxes and files are available.
%
% Usage:
%   >> setup_project
%
% After running, start the simulation with:
%   >> main_ros2

function setup_project()
    fprintf('\n');
    fprintf('╔════════════════════════════════════════════════╗\n');
    fprintf('║  MINESWEEPER ROBOT ROS2 - PROJECT SETUP        ║\n');
    fprintf('╚════════════════════════════════════════════════╝\n\n');
    
    % Get project root
    projectRoot = fileparts(mfilename('fullpath'));
    if isempty(projectRoot)
        projectRoot = pwd;
    end
    
    fprintf('📁 Project Root: %s\n\n', projectRoot);
    
    %% Add all paths
    fprintf('📂 Adding paths...\n');
    addpath(projectRoot);
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'scripts'));
    addpath(genpath(fullfile(projectRoot, 'scripts')));
    addpath(fullfile(projectRoot, 'models'));
    fprintf('   ✓ All project folders added to MATLAB path\n\n');
    
    %% Check required toolboxes
    fprintf('🔧 Checking toolboxes...\n');
    
    toolboxes = {
        'Simulink',      'simulink',   true;
        'Stateflow',     'stateflow',  true;
        'ROS Toolbox',   'robotics',   false;
        'Navigation',    'navigation', false
    };
    
    allRequired = true;
    for i = 1:size(toolboxes, 1)
        name = toolboxes{i, 1};
        product = toolboxes{i, 2};
        required = toolboxes{i, 3};
        
        if license('test', product)
            fprintf('   ✓ %s - Available\n', name);
        else
            if required
                fprintf('   ✗ %s - MISSING (Required)\n', name);
                allRequired = false;
            else
                fprintf('   ○ %s - Not available (Optional)\n', name);
            end
        end
    end
    fprintf('\n');
    
    %% Check key project files
    fprintf('📄 Checking project files...\n');
    
    keyFiles = {
        'main_ros2.m',                    'Main simulation script';
        'config/robot_params.m',          'Configuration parameters';
        'scripts/buildStateflowModel.m',  'Stateflow model builder';
        'scripts/core/OccupancyGridWorld.m', 'World class';
        'scripts/sensors/EKFSLAM.m',      'EKF-SLAM class';
        'scripts/planning/PathPlannerROS.m', 'Path planner';
    };
    
    allFiles = true;
    for i = 1:size(keyFiles, 1)
        filePath = fullfile(projectRoot, keyFiles{i, 1});
        desc = keyFiles{i, 2};
        
        if exist(filePath, 'file')
            fprintf('   ✓ %s\n', keyFiles{i, 1});
        else
            fprintf('   ✗ %s - MISSING\n', keyFiles{i, 1});
            allFiles = false;
        end
    end
    fprintf('\n');
    
    %% Load and display configuration
    fprintf('⚙️  Loading configuration...\n');
    try
        params = robot_params();
        fprintf('   Grid: %dx%d | Mines: %.0f%% | Obstacles: %d\n', ...
            params.world.grid_rows, params.world.grid_cols, ...
            params.world.mine_density*100, params.world.num_obstacles);
        fprintf('   Robot speed: %.1f m/s | Lidar range: %.1f m\n', ...
            params.robot.max_velocity, params.lidar.range);
        fprintf('   ✓ Configuration loaded successfully\n\n');
    catch ME
        fprintf('   ✗ Failed to load configuration: %s\n\n', ME.message);
    end
    
    %% Summary
    fprintf('════════════════════════════════════════════════\n');
    if allRequired && allFiles
        fprintf('  ✅ PROJECT READY!\n');
        fprintf('════════════════════════════════════════════════\n\n');
        fprintf('▶ To run the simulation:\n');
        fprintf('   >> main_ros2\n\n');
        fprintf('▶ To build Stateflow model:\n');
        fprintf('   >> buildStateflowModel\n\n');
        fprintf('▶ To edit configuration:\n');
        fprintf('   >> edit config/robot_params.m\n\n');
    else
        fprintf('  ⚠️  PROJECT HAS ISSUES\n');
        fprintf('════════════════════════════════════════════════\n\n');
        if ~allRequired
            fprintf('Some required toolboxes are missing.\n');
        end
        if ~allFiles
            fprintf('Some project files are missing.\n');
        end
        fprintf('The simulation may not work correctly.\n\n');
    end
end
