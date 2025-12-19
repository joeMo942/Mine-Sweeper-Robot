% main_ros2.m - OPTIMIZED Fast Minesweeper ROS2 Simulation
% Uses simple grid visualization for speed + ROS2 integration
%
% Usage: Run this script to start the simulation

%% Clear environment
clear; clc; close all;

fprintf('================================================\n');
fprintf('  MINESWEEPER ROBOT - FAST ROS2 SIMULATION\n');
fprintf('================================================\n\n');

%% Configuration
thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir);
addpath(fullfile(thisDir, '..', 'config'));
run('robot_params.m');

% Fast simulation parameters
dt = 0.005;              % Small timestep for quality
maxTime = 1000;          % Max time
speedMultiplier = 5;    % Process 5 steps per viz update

fprintf('Speed: %dx | Robot velocity: %.1f m/s\n\n', speedMultiplier, robot.max_velocity);

%% Step 1: Create Simple Grid World (10x10 like original)
fprintf('Creating world...\n');
gridRows = 5;
gridCols = 5;
cellSize = 1.0;
mineDensity = 0.1;
numObstacles = 4;  % Number of obstacles

% Create mine map (random placement)
numMines = round(gridRows * gridCols * mineDensity);
startPos = [1, 1];
endPos = [gridRows, gridCols];

mineMap = false(gridRows, gridCols);
placedMines = 0;
while placedMines < numMines
    r = randi(gridRows);
    c = randi(gridCols);
    if ~mineMap(r,c) && ~(r <= 2 && c <= 2)  % Avoid start zone
        mineMap(r,c) = true;
        placedMines = placedMines + 1;
    end
end

% Create obstacle map (blocks robot cannot pass through)
obstacleMap = false(gridRows, gridCols);
placedObstacles = 0;
while placedObstacles < numObstacles
    r = randi(gridRows);
    c = randi(gridCols);
    % Avoid start zone, end zone, and mines
    if ~obstacleMap(r,c) && ~mineMap(r,c) && ...
       ~(r <= 2 && c <= 2) && ~(r >= gridRows-1 && c >= gridCols-1)
        obstacleMap(r,c) = true;
        placedObstacles = placedObstacles + 1;
    end
end

% Track detected/marked mines  
detectedMap = false(gridRows, gridCols);
markedMap = false(gridRows, gridCols);

fprintf('World: %dx%d grid, %d mines, %d obstacles\n\n', gridRows, gridCols, numMines, numObstacles);

%% Step 2: Initialize ROS2
fprintf('Initializing ROS2...\n');
try
    node = ros2node('/minesweeper_fast');
    pubPose = ros2publisher(node, '/robot/pose', 'geometry_msgs/Pose2D');
    pubMine = ros2publisher(node, '/mine_alert', 'std_msgs/Bool');
    pubMap = ros2publisher(node, '/map', 'nav_msgs/OccupancyGrid');
    pubCmd = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
    ros2Active = true;
    fprintf('ROS2 initialized.\n\n');
catch
    ros2Active = false;
    fprintf('ROS2 not available - running in simulation mode.\n\n');
end

%% Step 3: Initialize Robot
robotPos = [0.5, 0.5];  % Start position
robotHeading = 0;
robotVel = robot.max_velocity;

%% Step 4: Generate Path (boustrophedon - avoid obstacles)
path = [];
direction = 1;
for row = 1:gridRows
    if direction == 1
        cols = 1:gridCols;
    else
        cols = gridCols:-1:1;
    end
    for col = cols
        % Skip obstacle cells
        if ~obstacleMap(row, col)
            path(end+1,:) = [(col-0.5)*cellSize, (row-0.5)*cellSize];
        end
    end
    direction = -direction;
end
waypointIdx = 1;
fprintf('Path: %d waypoints (avoiding %d obstacles)\n\n', size(path,1), numObstacles);

%% Step 5: Create Fast Visualization
fig = figure('Name', 'Minesweeper ROS2 (Fast)', 'NumberTitle', 'off', ...
            'Position', [100 100 900 750], 'Color', [0.15 0.15 0.15]);

% Draw initial grid
ax = axes('Position', [0.1 0.12 0.8 0.78]);
hold on;
axis equal;
xlim([0, gridCols*cellSize]);
ylim([0, gridRows*cellSize]);
set(ax, 'Color', [0.2 0.2 0.2], 'XColor', 'w', 'YColor', 'w');
xlabel('X (m)', 'Color', 'w'); ylabel('Y (m)', 'Color', 'w');

% Draw grid lines
for i = 0:gridCols
    plot([i i]*cellSize, [0 gridRows]*cellSize, 'Color', [0.4 0.4 0.4]);
end
for i = 0:gridRows
    plot([0 gridCols]*cellSize, [i i]*cellSize, 'Color', [0.4 0.4 0.4]);
end

% ===== OBSTACLES - Gray filled squares =====
[obsRows, obsCols] = find(obstacleMap);
for i = 1:length(obsRows)
    ox = (obsCols(i) - 1) * cellSize;
    oy = (obsRows(i) - 1) * cellSize;
    rectangle('Position', [ox+0.05, oy+0.05, cellSize-0.1, cellSize-0.1], ...
             'FaceColor', [0.5 0.5 0.5], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 2);
end
hObstacles = plot(NaN, NaN, 's', 'MarkerSize', 15, 'MarkerFaceColor', [0.5 0.5 0.5], ...
                 'MarkerEdgeColor', 'k', 'DisplayName', 'Obstacles');

% ===== PRE-DEFINED MINES - Show mines as RED X from the start =====
[mineRows, mineCols] = find(mineMap);
mineWorldX = (mineCols - 0.5) * cellSize;
mineWorldY = (mineRows - 0.5) * cellSize;
hMinesVisible = plot(mineWorldX, mineWorldY, 'rx', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Mines');

% Graphics handles
hRobot = plot(robotPos(1), robotPos(2), 'bo', 'MarkerSize', 20, ...
             'MarkerFaceColor', [0.2 0.6 1], 'LineWidth', 2, 'DisplayName', 'Robot');
hTrail = plot(NaN, NaN, 'c-', 'LineWidth', 1.5, 'DisplayName', 'Trail');

% Handle for detected mines - GREEN CIRCLES
hDetectedMines = plot(NaN, NaN, 'go', 'MarkerSize', 30, 'LineWidth', 4, 'DisplayName', 'Detected');

title('MINESWEEPER ROS2 SIMULATION', 'Color', 'w', 'FontSize', 14);
legend([hRobot, hMinesVisible, hObstacles, hDetectedMines], ...
       {'Robot', 'Mines', 'Obstacles', 'Detected'}, ...
       'Location', 'northeast', 'TextColor', 'w', 'Color', [0.2 0.2 0.2]);

% Status text
hStatus = uicontrol('Style', 'text', 'Position', [10 10 880 40], ...
    'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'w', ...
    'FontSize', 12, 'HorizontalAlignment', 'left');

fprintf('================================================\n');
fprintf('  STARTING SIMULATION (Press Ctrl+C to stop)\n');
fprintf('================================================\n\n');

%% Main Loop
trail = robotPos;
minesFound = 0;
time = 0;
step = 0;
goalTol = 0.15;

tic;
while time < maxTime && isvalid(fig)
    for substep = 1:speedMultiplier
        step = step + 1;
        
        % Get current waypoint
        if waypointIdx > size(path,1)
            break;
        end
        target = path(waypointIdx,:);
        
        % Move towards target
        toTarget = target - robotPos;
        dist = norm(toTarget);
        
        if dist < goalTol
            waypointIdx = waypointIdx + 1;
            continue;
        end
        
        % Compute velocity
        desiredHeading = atan2(toTarget(2), toTarget(1));
        headingError = wrapToPi(desiredHeading - robotHeading);
        
        if abs(headingError) > 0.1
            robotHeading = robotHeading + sign(headingError) * robot.max_angular_velocity * dt;
            v = 0.1 * robotVel;
        else
            v = min(robotVel, dist * 2);
        end
        
        % Update position
        robotPos(1) = robotPos(1) + v * cos(robotHeading) * dt;
        robotPos(2) = robotPos(2) + v * sin(robotHeading) * dt;
        robotHeading = wrapToPi(robotHeading);
        
        % Check for mine
        gridR = max(1, min(gridRows, floor(robotPos(2)/cellSize) + 1));
        gridC = max(1, min(gridCols, floor(robotPos(1)/cellSize) + 1));
        
        mineAlert = false;
        if mineMap(gridR, gridC) && ~markedMap(gridR, gridC)
            mineAlert = true;
            detectedMap(gridR, gridC) = true;
            markedMap(gridR, gridC) = true;
            minesFound = minesFound + 1;
            fprintf('[%.1fs] MINE at grid (%d,%d) - Total: %d/%d\n', ...
                   time, gridR, gridC, minesFound, numMines);
        end
        
        % Publish ROS2
        if ros2Active
            poseMsg = ros2message('geometry_msgs/Pose2D');
            poseMsg.x = robotPos(1);
            poseMsg.y = robotPos(2);
            poseMsg.theta = robotHeading;
            send(pubPose, poseMsg);
            
            mineMsg = ros2message('std_msgs/Bool');
            mineMsg.data = mineAlert;
            send(pubMine, mineMsg);
        end
        
        trail(end+1,:) = robotPos;
        time = time + dt;
    end
    
    % Check if path complete
    if waypointIdx > size(path,1)
        fprintf('\nPath complete!\n');
        break;
    end
    
    % Update visualization
    set(hRobot, 'XData', robotPos(1), 'YData', robotPos(2));
    set(hTrail, 'XData', trail(:,1), 'YData', trail(:,2));
    
    % Show detected mines as GREEN CIRCLES
    [mr, mc] = find(markedMap);
    if ~isempty(mr)
        detectedX = (mc - 0.5) * cellSize;
        detectedY = (mr - 0.5) * cellSize;
        set(hDetectedMines, 'XData', detectedX, 'YData', detectedY);
    end
    
    % Update status
    progress = waypointIdx / size(path,1) * 100;
    elapsed = toc;
    statusStr = sprintf('Time: %.1fs | State: EXPLORE | Mines: %d/%d | Progress: %.0f%% | FPS: %.0f', ...
                       time, minesFound, numMines, progress, step/elapsed);
    set(hStatus, 'String', statusStr);
    title(ax, sprintf('Mines: %d/%d | Pos: (%.1f, %.1f)', minesFound, numMines, robotPos(1), robotPos(2)), ...
          'Color', 'w', 'FontSize', 12);
    
    drawnow limitrate;
end

%% Results
fprintf('\n================================================\n');
fprintf('  SIMULATION COMPLETE\n');
fprintf('================================================\n');
fprintf('Time: %.1f s | Mines: %d/%d | Steps: %d\n', time, minesFound, numMines, step);
fprintf('================================================\n');

%% Calculate and Show Shortest Safe Path in SAME FIGURE
fprintf('\nCalculating shortest safe path...\n');

% Create combined obstacle map (mines + obstacles)
combinedObstacleMap = mineMap | obstacleMap;  % Avoid both mines and obstacles

% Start and end positions
startPos = [1, 1];
endPos = [gridRows, gridCols];

% Find shortest path using A* algorithm
shortestPath = findShortestPath(combinedObstacleMap, startPos, endPos);

% Draw path on the SAME figure (reuse existing figure)
figure(fig);  % Bring simulation figure to front
hold(ax, 'on');

% Clear previous elements and redraw with path

% Draw shortest path on existing figure
if ~isempty(shortestPath)
    % Draw path line (thick blue with white dashed overlay)
    pathX = (shortestPath(:, 2) - 0.5) * cellSize;
    pathY = (shortestPath(:, 1) - 0.5) * cellSize;
    hShortPath = plot(ax, pathX, pathY, 'm-', 'LineWidth', 4, 'DisplayName', 'Shortest Path');
    plot(ax, pathX, pathY, 'w--', 'LineWidth', 2);
    
    % Mark path waypoints
    plot(ax, pathX, pathY, 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
    
    % Mark start and end
    plot(ax, pathX(1), pathY(1), 'g^', 'MarkerSize', 20, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot(ax, pathX(end), pathY(end), 'rs', 'MarkerSize', 20, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    
    fprintf('Shortest path found: %d steps\n', size(shortestPath, 1));
    pathSteps = size(shortestPath, 1);
    pathExists = true;
else
    fprintf('*** NO PATH EXISTS! ***\n');
    pathSteps = 0;
    pathExists = false;
    
    % Show big NO PATH text on figure
    text(ax, gridCols*cellSize/2, gridRows*cellSize/2, 'NO PATH EXISTS!', ...
        'HorizontalAlignment', 'center', 'FontSize', 24, 'FontWeight', 'bold', ...
        'Color', 'r', 'BackgroundColor', 'w');
end

% Update title with path info
if pathExists
    title(ax, sprintf('COMPLETE - Shortest Path: %d steps | Mines: %d/%d', ...
          pathSteps, minesFound, numMines), 'Color', 'g', 'FontSize', 14);
else
    title(ax, sprintf('COMPLETE - NO PATH! | Mines: %d/%d', minesFound, numMines), ...
          'Color', 'r', 'FontSize', 14);
end

% Update status bar
set(hStatus, 'String', sprintf('DONE! Shortest Path: %d steps | Mines Found: %d/%d', ...
    pathSteps, minesFound, numMines));

hold(ax, 'off');
drawnow;

fprintf('================================================\n');

%% Helper Functions
function a = wrapToPi(a)
    while a > pi, a = a - 2*pi; end
    while a < -pi, a = a + 2*pi; end
end

function path = findShortestPath(obstacleMap, startPos, endPos)
    % A* pathfinding algorithm
    [rows, cols] = size(obstacleMap);
    
    % Check if start or end is blocked
    if obstacleMap(startPos(1), startPos(2)) || obstacleMap(endPos(1), endPos(2))
        path = [];
        return;
    end
    
    % Initialize
    openList = startPos;
    gScore = inf(rows, cols);
    gScore(startPos(1), startPos(2)) = 0;
    fScore = inf(rows, cols);
    fScore(startPos(1), startPos(2)) = heuristic(startPos, endPos);
    cameFrom = zeros(rows, cols, 2);
    
    % Directions: up, down, left, right
    directions = [-1 0; 1 0; 0 -1; 0 1];
    
    while ~isempty(openList)
        % Find node with lowest fScore
        [~, idx] = min(arrayfun(@(i) fScore(openList(i,1), openList(i,2)), 1:size(openList,1)));
        current = openList(idx, :);
        
        % Check if reached goal
        if isequal(current, endPos)
            path = reconstructPath(cameFrom, current);
            return;
        end
        
        % Remove from open list
        openList(idx, :) = [];
        
        % Check neighbors
        for d = 1:4
            neighbor = current + directions(d, :);
            
            % Check bounds
            if neighbor(1) < 1 || neighbor(1) > rows || ...
               neighbor(2) < 1 || neighbor(2) > cols
                continue;
            end
            
            % Check if obstacle
            if obstacleMap(neighbor(1), neighbor(2))
                continue;
            end
            
            % Calculate tentative gScore
            tentativeG = gScore(current(1), current(2)) + 1;
            
            if tentativeG < gScore(neighbor(1), neighbor(2))
                % Better path found
                cameFrom(neighbor(1), neighbor(2), :) = current;
                gScore(neighbor(1), neighbor(2)) = tentativeG;
                fScore(neighbor(1), neighbor(2)) = tentativeG + heuristic(neighbor, endPos);
                
                % Add to open list if not already there
                if ~any(all(openList == neighbor, 2))
                    openList(end+1, :) = neighbor;
                end
            end
        end
    end
    
    % No path found
    path = [];
end

function h = heuristic(pos, goal)
    % Manhattan distance
    h = abs(pos(1) - goal(1)) + abs(pos(2) - goal(2));
end

function path = reconstructPath(cameFrom, current)
    path = current;
    while any(cameFrom(current(1), current(2), :))
        current = squeeze(cameFrom(current(1), current(2), :))';
        if all(current == 0)
            break;
        end
        path = [current; path];
    end
end
