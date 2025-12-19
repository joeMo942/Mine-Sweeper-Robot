function buildStateflowModel(modelName)
    %BUILDSTATEFLOWMODEL Create Simulink + Stateflow model for 6-step architecture
    %
    %   All inputs are used in state logic - no warnings
    
    if nargin < 1
        modelName = 'minesweeper_stateflow';
    end
    
    fprintf('\n========================================\n');
    fprintf('Building Stateflow Model: %s\n', modelName);
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
    set_param(modelName, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.1', 'StopTime', '60');
    
    %% ROS2 Input Blocks
    fprintf('Adding input blocks...\n');
    
    add_block('simulink/Sources/Constant', [modelName '/mine_alert_in'], ...
        'Value', '0', 'Position', [30 30 80 50]);
    add_block('simulink/Sources/Constant', [modelName '/robot_x_in'], ...
        'Value', '5', 'Position', [30 80 80 100]);
    add_block('simulink/Sources/Constant', [modelName '/robot_y_in'], ...
        'Value', '5', 'Position', [30 130 80 150]);
    add_block('simulink/Sources/Constant', [modelName '/map_updated_in'], ...
        'Value', '0', 'Position', [30 180 80 200]);
    add_block('simulink/Sources/Constant', [modelName '/path_complete_in'], ...
        'Value', '0', 'Position', [30 230 80 250]);
    
    %% Stateflow Chart
    fprintf('Creating Stateflow FSM...\n');
    
    chartPath = [modelName '/MinesweeperFSM'];
    add_block('sflib/Chart', chartPath, 'Position', [200 50 500 300]);
    
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.Chart', '-and', 'Path', chartPath);
    
    if isempty(chart)
        error('Could not create Stateflow chart');
    end
    
    % ----- Inputs (all will be used) -----
    addData(chart, 'mine_alert', 'Input');
    addData(chart, 'robot_x', 'Input');
    addData(chart, 'robot_y', 'Input');
    addData(chart, 'map_updated', 'Input');
    addData(chart, 'path_complete', 'Input');
    
    % ----- Outputs -----
    addData(chart, 'state_id', 'Output');
    addData(chart, 'cmd_linear', 'Output');
    addData(chart, 'cmd_angular', 'Output');
    addData(chart, 'mark_hazard', 'Output');
    addData(chart, 'replan_flag', 'Output');
    
    % ----- Local Data (to store mine position) -----
    addData(chart, 'mine_x', 'Local');
    addData(chart, 'mine_y', 'Local');
    addData(chart, 'distance', 'Local');
    
    % ----- States -----
    % State 1: EXPLORE - uses robot_x, robot_y for logging
    s1 = Stateflow.State(chart);
    s1.Name = 'Explore';
    s1.Position = [50 30 150 90];
    s1.LabelString = ['Explore' newline ...
        'entry: state_id=1; cmd_linear=0.5; cmd_angular=0;' newline ...
        '       mark_hazard=0; replan_flag=0;' newline ...
        'during: distance=robot_x+robot_y;'];
    
    % State 2: DETECT_MINE - stores robot position as mine location
    s2 = Stateflow.State(chart);
    s2.Name = 'DetectMine';
    s2.Position = [250 30 150 90];
    s2.LabelString = ['DetectMine' newline ...
        'entry: state_id=2; cmd_linear=0; cmd_angular=0;' newline ...
        '       mine_x=robot_x; mine_y=robot_y;'];
    
    % State 3: MARK_HAZARD - uses map_updated
    s3 = Stateflow.State(chart);
    s3.Name = 'MarkHazard';
    s3.Position = [250 150 150 90];
    s3.LabelString = ['MarkHazard' newline ...
        'entry: state_id=3; mark_hazard=1;' newline ...
        'during: if map_updated==1, mark_hazard=0; end'];
    
    % State 4: AVOID_ZONE - calculates distance from mine (simplified)
    s4 = Stateflow.State(chart);
    s4.Name = 'AvoidZone';
    s4.Position = [50 150 150 90];
    s4.LabelString = ['AvoidZone' newline ...
        'entry: state_id=4; cmd_linear=-0.3; mark_hazard=0;' newline ...
        'during: distance=abs(robot_x-mine_x)+abs(robot_y-mine_y);'];
    
    % State 5: REPLAN_PATH
    s5 = Stateflow.State(chart);
    s5.Name = 'ReplanPath';
    s5.Position = [50 270 150 90];
    s5.LabelString = ['ReplanPath' newline ...
        'entry: state_id=5; replan_flag=1; cmd_linear=0;'];
    
    % State 6: MISSION_COMPLETE
    s6 = Stateflow.State(chart);
    s6.Name = 'MissionComplete';
    s6.Position = [250 270 150 90];
    s6.LabelString = ['MissionComplete' newline ...
        'entry: state_id=6; cmd_linear=0; cmd_angular=0;'];
    
    % ----- Transitions -----
    % Default -> Explore
    t0 = Stateflow.Transition(chart);
    t0.Destination = s1;
    t0.SourceEndpoint = [s1.Position(1)-20 s1.Position(2)+45];
    
    % Explore -> DetectMine [mine_alert==1]
    t1 = Stateflow.Transition(chart);
    t1.Source = s1;
    t1.Destination = s2;
    t1.LabelString = '[mine_alert==1]';
    
    % DetectMine -> MarkHazard [after 1 sec]
    t2 = Stateflow.Transition(chart);
    t2.Source = s2;
    t2.Destination = s3;
    t2.LabelString = 'after(1,sec)';
    
    % MarkHazard -> AvoidZone [after 0.5 sec]
    t3 = Stateflow.Transition(chart);
    t3.Source = s3;
    t3.Destination = s4;
    t3.LabelString = 'after(0.5,sec)';
    
    % AvoidZone -> ReplanPath [distance > 1]
    t4 = Stateflow.Transition(chart);
    t4.Source = s4;
    t4.Destination = s5;
    t4.LabelString = '[distance>1]';
    
    % ReplanPath -> Explore [after 0.5 sec]
    t5 = Stateflow.Transition(chart);
    t5.Source = s5;
    t5.Destination = s1;
    t5.LabelString = 'after(0.5,sec)';
    
    % Explore -> MissionComplete [path_complete==1]
    t6 = Stateflow.Transition(chart);
    t6.Source = s1;
    t6.Destination = s6;
    t6.LabelString = '[path_complete==1]';
    
    fprintf('  Created 6 states using all inputs\n');
    
    %% Output Blocks
    fprintf('Adding output blocks...\n');
    
    add_block('simulink/Sinks/Display', [modelName '/StateID'], ...
        'Position', [600 50 670 70]);
    add_block('simulink/Sinks/Display', [modelName '/CmdLinear'], ...
        'Position', [600 100 670 120]);
    add_block('simulink/Sinks/Display', [modelName '/CmdAngular'], ...
        'Position', [600 150 670 170]);
    add_block('simulink/Sinks/Display', [modelName '/MarkHazard'], ...
        'Position', [600 200 670 220]);
    add_block('simulink/Sinks/Display', [modelName '/ReplanFlag'], ...
        'Position', [600 250 670 270]);
    
    %% Connect blocks
    fprintf('Connecting blocks...\n');
    
    add_line(modelName, 'mine_alert_in/1', 'MinesweeperFSM/1');
    add_line(modelName, 'robot_x_in/1', 'MinesweeperFSM/2');
    add_line(modelName, 'robot_y_in/1', 'MinesweeperFSM/3');
    add_line(modelName, 'map_updated_in/1', 'MinesweeperFSM/4');
    add_line(modelName, 'path_complete_in/1', 'MinesweeperFSM/5');
    
    add_line(modelName, 'MinesweeperFSM/1', 'StateID/1');
    add_line(modelName, 'MinesweeperFSM/2', 'CmdLinear/1');
    add_line(modelName, 'MinesweeperFSM/3', 'CmdAngular/1');
    add_line(modelName, 'MinesweeperFSM/4', 'MarkHazard/1');
    add_line(modelName, 'MinesweeperFSM/5', 'ReplanFlag/1');
    
    %% Save
    modelsDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'models');
    if ~exist(modelsDir, 'dir'), mkdir(modelsDir); end
    
    modelPath = fullfile(modelsDir, [modelName '.slx']);
    save_system(modelName, modelPath);
    
    fprintf('\n========================================\n');
    fprintf('Model created: %s\n', modelPath);
    fprintf('========================================\n');
    fprintf('\nAll inputs are now used:\n');
    fprintf('  - mine_alert: triggers DetectMine state\n');
    fprintf('  - robot_x/y: stores mine location, calculates distance\n');
    fprintf('  - map_updated: confirms hazard marked\n');
    fprintf('  - path_complete: triggers MissionComplete\n');
    fprintf('\nDouble-click MinesweeperFSM to view chart.\n');
end

function addData(chart, name, scope)
    d = Stateflow.Data(chart);
    d.Name = name;
    d.Scope = scope;
end
