function buildSimulinkModel(modelName)
    %BUILDSIMULINKMODEL Create Simulink model for minesweeper controller
    %   buildSimulinkModel() - creates with default name
    %   buildSimulinkModel(modelName) - creates with specified name
    %
    %   This function programmatically creates a Simulink model for the
    %   minesweeper robot controller with ROS2 integration.
    
    if nargin < 1
        modelName = 'minesweeper_controller';
    end
    
    fprintf('Building Simulink model: %s\n', modelName);
    
    %% Check for Simulink
    if ~license('test', 'Simulink')
        error('Simulink license not found. Cannot create model.');
    end
    
    %% Close existing model if open
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    
    %% Create new model
    fprintf('Creating new Simulink model...\n');
    new_system(modelName);
    open_system(modelName);
    
    %% Set model configuration
    fprintf('Configuring model parameters...\n');
    
    % Solver settings
    set_param(modelName, 'Solver', 'FixedStepDiscrete');
    set_param(modelName, 'FixedStep', '0.1');
    set_param(modelName, 'StopTime', '300');
    
    % Hardware settings for ROS2 (if available)
    try
        set_param(modelName, 'HardwareBoard', 'Robot Operating System 2 (ROS 2)');
    catch
        warning('ROS2 hardware board not available. Using default settings.');
    end
    
    %% Add Subsystems
    fprintf('Adding subsystems...\n');
    
    % Subsystem positions
    inputSubPos = [100, 100, 250, 200];
    controllerSubPos = [350, 100, 500, 200];
    outputSubPos = [600, 100, 750, 200];
    
    % Input Subsystem (ROS2 Subscribers / Simulated Inputs)
    add_block('simulink/Ports & Subsystems/Subsystem', ...
              [modelName, '/ROS2_Input'], ...
              'Position', inputSubPos);
    
    % Controller Subsystem
    add_block('simulink/Ports & Subsystems/Subsystem', ...
              [modelName, '/Controller'], ...
              'Position', controllerSubPos);
    
    % Output Subsystem (ROS2 Publishers / Outputs)
    add_block('simulink/Ports & Subsystems/Subsystem', ...
              [modelName, '/ROS2_Output'], ...
              'Position', outputSubPos);
    
    %% Build Input Subsystem
    fprintf('Building Input subsystem...\n');
    inputSub = [modelName, '/ROS2_Input'];
    
    % Delete default content
    delete_block([inputSub, '/In1']);
    delete_block([inputSub, '/Out1']);
    
    % Add simulated pose input (Constant for now)
    add_block('simulink/Sources/Constant', [inputSub, '/Pose_X'], ...
              'Position', [50, 30, 80, 60], 'Value', '0');
    add_block('simulink/Sources/Constant', [inputSub, '/Pose_Y'], ...
              'Position', [50, 80, 80, 110], 'Value', '0');
    add_block('simulink/Sources/Constant', [inputSub, '/Pose_Theta'], ...
              'Position', [50, 130, 80, 160], 'Value', '0');
    add_block('simulink/Sources/Constant', [inputSub, '/Mine_Detected'], ...
              'Position', [50, 180, 80, 210], 'Value', '0');
    
    % Add Mux to combine pose
    add_block('simulink/Signal Routing/Mux', [inputSub, '/Pose_Mux'], ...
              'Position', [150, 50, 155, 150], 'Inputs', '3');
    
    % Add output ports
    add_block('simulink/Ports & Subsystems/Out1', [inputSub, '/Pose'], ...
              'Position', [250, 90, 280, 110]);
    add_block('simulink/Ports & Subsystems/Out1', [inputSub, '/MineDetected'], ...
              'Position', [250, 185, 280, 205]);
    
    % Connect blocks in input subsystem
    add_line(inputSub, 'Pose_X/1', 'Pose_Mux/1');
    add_line(inputSub, 'Pose_Y/1', 'Pose_Mux/2');
    add_line(inputSub, 'Pose_Theta/1', 'Pose_Mux/3');
    add_line(inputSub, 'Pose_Mux/1', 'Pose/1');
    add_line(inputSub, 'Mine_Detected/1', 'MineDetected/1');
    
    %% Build Controller Subsystem
    fprintf('Building Controller subsystem...\n');
    ctrlSub = [modelName, '/Controller'];
    
    % Delete default content
    delete_block([ctrlSub, '/In1']);
    delete_block([ctrlSub, '/Out1']);
    
    % Add input ports
    add_block('simulink/Ports & Subsystems/In1', [ctrlSub, '/Pose'], ...
              'Position', [30, 70, 60, 90]);
    add_block('simulink/Ports & Subsystems/In1', [ctrlSub, '/MineDetected'], ...
              'Position', [30, 140, 60, 160]);
    add_block('simulink/Ports & Subsystems/In1', [ctrlSub, '/Target'], ...
              'Position', [30, 210, 60, 230]);
    
    % Add MATLAB Function block for controller
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [ctrlSub, '/ControllerFcn'], ...
              'Position', [150, 100, 280, 180]);
    
    % Add output port
    add_block('simulink/Ports & Subsystems/Out1', [ctrlSub, '/CmdVel'], ...
              'Position', [350, 130, 380, 150]);
    
    % Connect blocks
    add_line(ctrlSub, 'Pose/1', 'ControllerFcn/1');
    add_line(ctrlSub, 'MineDetected/1', 'ControllerFcn/2');
    add_line(ctrlSub, 'Target/1', 'ControllerFcn/3');
    add_line(ctrlSub, 'ControllerFcn/1', 'CmdVel/1');
    
    %% Build Output Subsystem
    fprintf('Building Output subsystem...\n');
    outSub = [modelName, '/ROS2_Output'];
    
    % Delete default content
    delete_block([outSub, '/In1']);
    delete_block([outSub, '/Out1']);
    
    % Add input port
    add_block('simulink/Ports & Subsystems/In1', [outSub, '/CmdVel'], ...
              'Position', [50, 100, 80, 120]);
    
    % Add demux for velocity components
    add_block('simulink/Signal Routing/Demux', [outSub, '/Vel_Demux'], ...
              'Position', [150, 85, 155, 135], 'Outputs', '2');
    
    % Add display blocks
    add_block('simulink/Sinks/Display', [outSub, '/Linear_Vel'], ...
              'Position', [250, 70, 320, 100]);
    add_block('simulink/Sinks/Display', [outSub, '/Angular_Vel'], ...
              'Position', [250, 120, 320, 150]);
    
    % Add To Workspace block for logging
    add_block('simulink/Sinks/To Workspace', [outSub, '/CmdVel_Log'], ...
              'Position', [250, 180, 320, 210], ...
              'VariableName', 'cmd_vel_log');
    
    % Connect blocks
    add_line(outSub, 'CmdVel/1', 'Vel_Demux/1');
    add_line(outSub, 'Vel_Demux/1', 'Linear_Vel/1');
    add_line(outSub, 'Vel_Demux/2', 'Angular_Vel/1');
    add_line(outSub, 'CmdVel/1', 'CmdVel_Log/1');
    
    %% Connect Main Subsystems
    fprintf('Connecting main subsystems...\n');
    
    % Add target input (constant for now)
    add_block('simulink/Sources/Constant', [modelName, '/Target_Position'], ...
              'Position', [250, 200, 300, 230], 'Value', '[5, 5]');
    
    % Add connections between main subsystems
    add_line(modelName, 'ROS2_Input/1', 'Controller/1');
    add_line(modelName, 'ROS2_Input/2', 'Controller/2');
    add_line(modelName, 'Target_Position/1', 'Controller/3');
    add_line(modelName, 'Controller/1', 'ROS2_Output/1');
    
    %% Add Annotations
    fprintf('Adding annotations...\n');
    
    % Title annotation
    add_block('simulink/Annotations/Note', [modelName, '/Title'], ...
              'Position', [300, 20], ...
              'Text', 'Minesweeper Robot Controller with ROS2');
    
    %% Save model
    fprintf('Saving model...\n');
    
    % Get the models directory
    projectRoot = getappdata(0, 'MinesweeperProjectRoot');
    if isempty(projectRoot)
        projectRoot = fileparts(fileparts(mfilename('fullpath')));
    end
    
    modelsDir = fullfile(projectRoot, 'models');
    if ~exist(modelsDir, 'dir')
        mkdir(modelsDir);
    end
    
    modelPath = fullfile(modelsDir, [modelName, '.slx']);
    save_system(modelName, modelPath);
    
    fprintf('\n========================================\n');
    fprintf('Simulink model created successfully!\n');
    fprintf('Model saved to: %s\n', modelPath);
    fprintf('========================================\n\n');
    
    fprintf('To open the model, run:\n');
    fprintf('  open_system(''%s'')\n\n', modelName);
end
