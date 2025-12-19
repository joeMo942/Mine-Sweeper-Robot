% setup_project.m - Project Initialization Script
% Minesweeper Robot Project with MATLAB Simulink and ROS2

function setup_project()
    %SETUP_PROJECT Initialize the minesweeper project environment
    %   This function sets up paths, checks dependencies, and loads parameters
    
    fprintf('=== Minesweeper Robot Project Setup ===\n\n');
    
    %% Get project root directory
    thisFile = mfilename('fullpath');
    scriptsDir = fileparts(thisFile);
    projectRoot = fileparts(scriptsDir);
    
    %% Add project paths
    fprintf('Adding project paths...\n');
    addpath(fullfile(projectRoot, 'scripts'));
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'models'));
    addpath(fullfile(projectRoot, 'results'));
    
    %% Create results directory if it doesn't exist
    resultsDir = fullfile(projectRoot, 'results');
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
        fprintf('Created results directory: %s\n', resultsDir);
    end
    
    %% Check for required toolboxes
    fprintf('\nChecking required toolboxes...\n');
    
    % Check for Simulink
    if ~license('test', 'Simulink')
        warning('Simulink license not found. Some features may not work.');
    else
        fprintf('  [OK] Simulink\n');
    end
    
    % Check for ROS Toolbox
    hasROS = false;
    try
        % Try to check if ROS Toolbox is available
        if exist('ros2node', 'file') || exist('ros2', 'file')
            hasROS = true;
            fprintf('  [OK] ROS Toolbox\n');
        else
            warning('ROS Toolbox not found. ROS2 features will be simulated.');
        end
    catch
        warning('ROS Toolbox not found. ROS2 features will be simulated.');
    end
    
    % Check for Stateflow (optional)
    if license('test', 'Stateflow')
        fprintf('  [OK] Stateflow (optional)\n');
    else
        fprintf('  [--] Stateflow not available (optional)\n');
    end
    
    %% Load robot parameters
    fprintf('\nLoading robot parameters...\n');
    run(fullfile(projectRoot, 'config', 'robot_params.m'));
    
    %% Set global configuration
    setappdata(0, 'MinesweeperProjectRoot', projectRoot);
    setappdata(0, 'MinesweeperHasROS', hasROS);
    
    %% Display project info
    fprintf('\n=== Setup Complete ===\n');
    fprintf('Project Root: %s\n', projectRoot);
    fprintf('ROS2 Support: %s\n', string(hasROS));
    fprintf('\nTo start the simulation, run: main\n');
    fprintf('==========================================\n\n');
end
