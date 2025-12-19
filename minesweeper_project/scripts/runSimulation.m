function results = runSimulation(robot, field, ros2if, viz, planner, params)
    %RUNSIMULATION Main simulation loop for minesweeper robot
    %   results = runSimulation(robot, field, ros2if, viz, planner)
    %   results = runSimulation(robot, field, ros2if, viz, planner, params)
    %
    %   Inputs:
    %       robot   - MinesweeperRobot instance
    %       field   - MinefieldGenerator instance
    %       ros2if  - ROS2Interface instance
    %       viz     - Visualization instance
    %       planner - PathPlanner instance
    %       params  - Optional simulation parameters struct
    %
    %   Outputs:
    %       results - Simulation results structure

    %% Setup parameters
    if nargin < 6
        params = struct();
    end
    
    % Default parameters
    if ~isfield(params, 'dt')
        params.dt = 0.1;  % Time step (seconds)
    end
    if ~isfield(params, 'maxTime')
        params.maxTime = 300;  % Maximum simulation time (seconds)
    end
    if ~isfield(params, 'vizUpdateRate')
        params.vizUpdateRate = 10;  % Visualization update rate (Hz)
    end
    if ~isfield(params, 'logEnabled')
        params.logEnabled = true;
    end
    
    %% Initialize simulation
    fprintf('\n========================================\n');
    fprintf('   MINESWEEPER ROBOT SIMULATION\n');
    fprintf('========================================\n\n');
    
    time = 0;
    step = 0;
    vizUpdateCounter = 0;
    vizUpdateInterval = round(1 / (params.vizUpdateRate * params.dt));
    
    % Generate initial path
    fprintf('Generating coverage path...\n');
    planner.planPath(robot.position, field);
    fprintf('Path generated with %d waypoints\n\n', planner.getNumWaypoints());
    
    % Initialize results
    results = struct();
    results.startTime = datetime('now');
    results.timeHistory = [];
    results.positionHistory = [];
    results.stateHistory = {};
    results.minesDetected = 0;
    results.minesMarked = 0;
    results.pathCompleted = false;
    results.simulationComplete = false;
    
    % Publish initial status
    ros2if.publishSimulationStatus('STARTED');
    
    fprintf('Starting simulation (max time: %.0f seconds)...\n\n', params.maxTime);
    
    %% Main simulation loop
    while time < params.maxTime
        step = step + 1;
        
        %% State machine logic
        switch robot.state
            case MinesweeperRobot.STATE_IDLE
                % Get next waypoint and start moving
                nextWaypoint = planner.getNextWaypoint();
                if ~isempty(nextWaypoint)
                    robot.setTarget(nextWaypoint);
                else
                    % Path complete
                    results.pathCompleted = true;
                    break;
                end
                
            case MinesweeperRobot.STATE_MOVING
                % Continue moving - update handled by robot.update()
                
            case MinesweeperRobot.STATE_SCANNING
                % Arrived at waypoint, scan for mines
                if robot.mineDetected
                    robot.setState(MinesweeperRobot.STATE_MARKING);
                else
                    % Move to next waypoint
                    planner.advanceWaypoint();
                    robot.setState(MinesweeperRobot.STATE_IDLE);
                end
                
            case MinesweeperRobot.STATE_MARKING
                % Mark the detected mine
                robot.markCurrentMine(field);
                results.minesMarked = size(robot.markedMines, 1);
                
                % Log event
                if params.logEnabled
                    fprintf('[%.1fs] Mine marked at (%.1f, %.1f) - Total: %d\n', ...
                           time, robot.position(1), robot.position(2), results.minesMarked);
                end
                
                % Continue to next waypoint
                planner.advanceWaypoint();
                robot.setState(MinesweeperRobot.STATE_IDLE);
                
            case MinesweeperRobot.STATE_AVOIDING
                % Obstacle avoidance (simplified)
                robot.setState(MinesweeperRobot.STATE_IDLE);
        end
        
        %% Update robot state
        robot.update(params.dt, field);
        
        %% Update ROS2 interface
        ros2if.updateFromRobot(robot);
        
        %% Update visualization
        vizUpdateCounter = vizUpdateCounter + 1;
        if vizUpdateCounter >= vizUpdateInterval
            viz.update(robot, planner);
            vizUpdateCounter = 0;
            
            % Check if figure was closed
            if ~isvalid(viz.fig)
                fprintf('\nVisualization closed by user. Stopping simulation.\n');
                break;
            end
        end
        
        %% Record history
        results.timeHistory(end+1) = time;
        results.positionHistory(end+1, :) = robot.position;
        results.stateHistory{end+1} = robot.state;
        results.minesDetected = size(robot.detectedMines, 1);
        
        %% Check completion
        [complete, stats] = field.getCompletionStatus();
        if complete
            fprintf('\n*** ALL MINES FOUND! ***\n');
            results.simulationComplete = true;
            break;
        end
        
        %% Advance time
        time = time + params.dt;
        
        %% Progress reporting (every 10 seconds)
        if mod(step, round(10 / params.dt)) == 0
            progress = planner.getProgress() * 100;
            fprintf('[%.0fs] Progress: %.1f%% | Position: (%.1f, %.1f) | Mines: %d/%d\n', ...
                   time, progress, robot.position(1), robot.position(2), ...
                   stats.markedMines, stats.totalMines);
        end
    end
    
    %% Simulation complete
    results.endTime = datetime('now');
    results.totalTime = time;
    results.finalStats = field.getCompletionStatus();
    
    % Publish final status
    ros2if.publishSimulationStatus('COMPLETED');
    
    % Show all mines at the end
    viz.showAllMines();
    
    %% Print summary
    fprintf('\n========================================\n');
    fprintf('   SIMULATION COMPLETE\n');
    fprintf('========================================\n');
    fprintf('Total time: %.1f seconds\n', time);
    fprintf('Steps executed: %d\n', step);
    fprintf('Path progress: %.1f%%\n', planner.getProgress() * 100);
    fprintf('Mines detected: %d\n', results.minesDetected);
    fprintf('Mines marked: %d / %d\n', results.minesMarked, stats.totalMines);
    fprintf('Distance traveled: %.1f m\n', calculatePathLength(results.positionHistory));
    
    if results.simulationComplete
        fprintf('\n*** SUCCESS: All mines found! ***\n');
    elseif results.pathCompleted
        fprintf('\n[INFO] Path completed. Some mines may not be found.\n');
    else
        fprintf('\n[INFO] Simulation ended (time limit or user interrupt).\n');
    end
    fprintf('========================================\n\n');
    
    %% Save results
    try
        resultsFile = fullfile(fileparts(mfilename('fullpath')), '..', 'results', ...
                              ['simulation_', datestr(now, 'yyyymmdd_HHMMSS'), '.mat']);
        save(resultsFile, 'results');
        fprintf('Results saved to: %s\n', resultsFile);
    catch ME
        warning('Failed to save results: %s', ME.message);
    end
end

function pathLength = calculatePathLength(positions)
    %CALCULATEPATHLENGTH Calculate total path length
    if size(positions, 1) < 2
        pathLength = 0;
        return;
    end
    
    diffs = diff(positions);
    distances = sqrt(sum(diffs.^2, 2));
    pathLength = sum(distances);
end
