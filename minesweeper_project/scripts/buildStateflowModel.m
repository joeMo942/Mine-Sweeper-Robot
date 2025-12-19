function buildStateflowModel(modelName)
    %BUILDSTATEFLOWMODEL Create Simulink + Stateflow model with SLAM and Dynamic Path Planning
    %
    %   Updated for 6-step architecture with SLAM visualization and live path updates
    
    if nargin < 1
        modelName = 'minesweeper_stateflow';
    end
    
    fprintf('\n========================================\n');
    fprintf('Building Stateflow Model: %s\n', modelName);
    fprintf('With SLAM + Dynamic Path Planning\n');
    fprintf('========================================\n\n');
    
    %% Check toolboxes
    assert(license('test', 'Simulink'), 'Simulink not available');
    assert(license('test', 'Stateflow'), 'Stateflow not available');
    
    %% Close if exists
    if bdIsLoaded(modelName)
        try
            set_param(modelName, 'SimulationCommand', 'stop');
            pause(0.5);
        catch
        end
        close_system(modelName, 0);
    end
    
    %% Create model
    new_system(modelName);
    open_system(modelName);
    set_param(modelName, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.1', 'StopTime', '120');
    
    %% ROS2 Input Blocks
    fprintf('Adding input blocks (sensors)...\n');
    
    add_block('simulink/Sources/Constant', [modelName '/mine_alert_in'], ...
        'Value', '0', 'Position', [30 30 80 50]);
    add_block('simulink/Sources/Constant', [modelName '/robot_x_in'], ...
        'Value', '0.5', 'Position', [30 80 80 100]);
    add_block('simulink/Sources/Constant', [modelName '/robot_y_in'], ...
        'Value', '0.5', 'Position', [30 130 80 150]);
    add_block('simulink/Sources/Constant', [modelName '/obstacle_detected_in'], ...
        'Value', '0', 'Position', [30 180 80 200]);
    add_block('simulink/Sources/Constant', [modelName '/path_complete_in'], ...
        'Value', '0', 'Position', [30 230 80 250]);
    add_block('simulink/Sources/Constant', [modelName '/slam_updated_in'], ...
        'Value', '0', 'Position', [30 280 80 300]);
    
    %% Stateflow Chart
    fprintf('Creating Stateflow FSM with SLAM states...\n');
    
    chartPath = [modelName '/MinesweeperFSM'];
    add_block('sflib/Chart', chartPath, 'Position', [200 50 550 350]);
    
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.Chart', '-and', 'Path', chartPath);
    
    if isempty(chart)
        error('Could not create Stateflow chart');
    end
    
    % ----- Inputs -----
    addData(chart, 'mine_alert', 'Input');
    addData(chart, 'robot_x', 'Input');
    addData(chart, 'robot_y', 'Input');
    addData(chart, 'obstacle_detected', 'Input');
    addData(chart, 'path_complete', 'Input');
    addData(chart, 'slam_updated', 'Input');
    
    % ----- Outputs -----
    addData(chart, 'state_id', 'Output');
    addData(chart, 'cmd_linear', 'Output');
    addData(chart, 'cmd_angular', 'Output');
    addData(chart, 'mark_hazard', 'Output');
    addData(chart, 'replan_flag', 'Output');
    addData(chart, 'slam_update_trigger', 'Output');
    
    % ----- Local Data -----
    addData(chart, 'obstacle_x', 'Local');
    addData(chart, 'obstacle_y', 'Local');
    addData(chart, 'mine_x', 'Local');
    addData(chart, 'mine_y', 'Local');
    addData(chart, 'distance', 'Local');
    addData(chart, 'known_obstacles', 'Local');
    addData(chart, 'known_mines', 'Local');
    
    % ===== STATES =====
    
    % State 1: EXPLORE - Main exploration state
    s1 = Stateflow.State(chart);
    s1.Name = 'Explore';
    s1.Position = [50 30 160 100];
    s1.LabelString = ['Explore' newline ...
        'entry: state_id=1; cmd_linear=0.5; cmd_angular=0;' newline ...
        '       mark_hazard=0; replan_flag=0;' newline ...
        'during: distance=robot_x+robot_y;'];
    
    % State 2: DETECT_OBSTACLE - Sensor detected obstacle
    s2 = Stateflow.State(chart);
    s2.Name = 'DetectObstacle';
    s2.Position = [270 30 160 100];
    s2.LabelString = ['DetectObstacle' newline ...
        'entry: state_id=2; cmd_linear=0;' newline ...
        '       obstacle_x=robot_x; obstacle_y=robot_y;' newline ...
        '       known_obstacles=known_obstacles+1;'];
    
    % State 3: UPDATE_SLAM - Update SLAM map with discovery
    s3 = Stateflow.State(chart);
    s3.Name = 'UpdateSLAM';
    s3.Position = [270 160 160 100];
    s3.LabelString = ['UpdateSLAM' newline ...
        'entry: state_id=3; slam_update_trigger=1;' newline ...
        'during: if slam_updated==1, slam_update_trigger=0; end'];
    
    % State 4: REPLAN_PATH - A* path replanning
    s4 = Stateflow.State(chart);
    s4.Name = 'ReplanPath';
    s4.Position = [50 160 160 100];
    s4.LabelString = ['ReplanPath' newline ...
        'entry: state_id=4; replan_flag=1; cmd_linear=0;' newline ...
        'during: distance=abs(robot_x-obstacle_x)+abs(robot_y-obstacle_y);'];
    
    % State 5: DETECT_MINE - Mine detected by sensor
    s5 = Stateflow.State(chart);
    s5.Name = 'DetectMine';
    s5.Position = [50 290 160 100];
    s5.LabelString = ['DetectMine' newline ...
        'entry: state_id=5; cmd_linear=0; mark_hazard=1;' newline ...
        '       mine_x=robot_x; mine_y=robot_y;' newline ...
        '       known_mines=known_mines+1;'];
    
    % State 6: MISSION_COMPLETE - All cells explored
    s6 = Stateflow.State(chart);
    s6.Name = 'MissionComplete';
    s6.Position = [270 290 160 100];
    s6.LabelString = ['MissionComplete' newline ...
        'entry: state_id=6; cmd_linear=0; cmd_angular=0;' newline ...
        '       replan_flag=0; mark_hazard=0;'];
    
    % ===== TRANSITIONS =====
    
    % Default -> Explore
    t0 = Stateflow.Transition(chart);
    t0.Destination = s1;
    t0.SourceEndpoint = [s1.Position(1)-20 s1.Position(2)+50];
    
    % Explore -> DetectObstacle [obstacle_detected==1]
    t1 = Stateflow.Transition(chart);
    t1.Source = s1;
    t1.Destination = s2;
    t1.LabelString = '[obstacle_detected==1]';
    
    % DetectObstacle -> UpdateSLAM [after 0.2 sec]
    t2 = Stateflow.Transition(chart);
    t2.Source = s2;
    t2.Destination = s3;
    t2.LabelString = 'after(0.2,sec)';
    
    % UpdateSLAM -> ReplanPath [after 0.3 sec]
    t3 = Stateflow.Transition(chart);
    t3.Source = s3;
    t3.Destination = s4;
    t3.LabelString = 'after(0.3,sec)';
    
    % ReplanPath -> Explore [after 0.5 sec]
    t4 = Stateflow.Transition(chart);
    t4.Source = s4;
    t4.Destination = s1;
    t4.LabelString = 'after(0.5,sec)';
    
    % Explore -> DetectMine [mine_alert==1]
    t5 = Stateflow.Transition(chart);
    t5.Source = s1;
    t5.Destination = s5;
    t5.LabelString = '[mine_alert==1]';
    
    % DetectMine -> UpdateSLAM [after 0.5 sec]
    t6 = Stateflow.Transition(chart);
    t6.Source = s5;
    t6.Destination = s3;
    t6.LabelString = 'after(0.5,sec)';
    
    % Explore -> MissionComplete [path_complete==1]
    t7 = Stateflow.Transition(chart);
    t7.Source = s1;
    t7.Destination = s6;
    t7.LabelString = '[path_complete==1]';
    
    fprintf('  Created 6 states with SLAM integration\n');
    
    %% Output Blocks
    fprintf('Adding output blocks...\n');
    
    add_block('simulink/Sinks/Display', [modelName '/StateID'], ...
        'Position', [650 50 720 70]);
    add_block('simulink/Sinks/Display', [modelName '/CmdLinear'], ...
        'Position', [650 100 720 120]);
    add_block('simulink/Sinks/Display', [modelName '/CmdAngular'], ...
        'Position', [650 150 720 170]);
    add_block('simulink/Sinks/Display', [modelName '/MarkHazard'], ...
        'Position', [650 200 720 220]);
    add_block('simulink/Sinks/Display', [modelName '/ReplanFlag'], ...
        'Position', [650 250 720 270]);
    add_block('simulink/Sinks/Display', [modelName '/SLAMTrigger'], ...
        'Position', [650 300 720 320]);
    
    %% Connect blocks
    fprintf('Connecting blocks...\n');
    
    add_line(modelName, 'mine_alert_in/1', 'MinesweeperFSM/1');
    add_line(modelName, 'robot_x_in/1', 'MinesweeperFSM/2');
    add_line(modelName, 'robot_y_in/1', 'MinesweeperFSM/3');
    add_line(modelName, 'obstacle_detected_in/1', 'MinesweeperFSM/4');
    add_line(modelName, 'path_complete_in/1', 'MinesweeperFSM/5');
    add_line(modelName, 'slam_updated_in/1', 'MinesweeperFSM/6');
    
    add_line(modelName, 'MinesweeperFSM/1', 'StateID/1');
    add_line(modelName, 'MinesweeperFSM/2', 'CmdLinear/1');
    add_line(modelName, 'MinesweeperFSM/3', 'CmdAngular/1');
    add_line(modelName, 'MinesweeperFSM/4', 'MarkHazard/1');
    add_line(modelName, 'MinesweeperFSM/5', 'ReplanFlag/1');
    add_line(modelName, 'MinesweeperFSM/6', 'SLAMTrigger/1');
    
    %% Save
    modelsDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'models');
    if ~exist(modelsDir, 'dir'), mkdir(modelsDir); end
    
    modelPath = fullfile(modelsDir, [modelName '.slx']);
    save_system(modelName, modelPath);
    
    fprintf('\n========================================\n');
    fprintf('Model created: %s\n', modelPath);
    fprintf('========================================\n');
    fprintf('\nStateflow States:\n');
    fprintf('  1. Explore - Main exploration with A* path following\n');
    fprintf('  2. DetectObstacle - Sensor detects obstacle\n');
    fprintf('  3. UpdateSLAM - Add discovery to SLAM map\n');
    fprintf('  4. ReplanPath - Recalculate optimal path with A*\n');
    fprintf('  5. DetectMine - Mine sensor triggered\n');
    fprintf('  6. MissionComplete - All cells explored\n');
    fprintf('\nDouble-click MinesweeperFSM to view chart.\n');
end

function addData(chart, name, scope)
    d = Stateflow.Data(chart);
    d.Name = name;
    d.Scope = scope;
end
