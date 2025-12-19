% main_ros2.m - Minesweeper Robot ROS2 Simulation with SLAM
%
% Main entry point for the simulation.
% All configuration is loaded from: config/robot_params.m
%
% Usage: 
%   >> main_ros2

%% Clear environment
clear; clc; close all;

fprintf('================================================\n');
fprintf('  MINESWEEPER ROBOT - ROS2 SIMULATION WITH SLAM\n');
fprintf('================================================\n\n');

%% Load Configuration from robot_params.m
projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot));  % Add all project folders

fprintf('Loading configuration...\n');
params = robot_params();  % Load all parameters from config file

% Extract parameters for convenience
robot = params.robot;
world = params.world;
sim = params.sim;
lidar = params.lidar;
viz = params.viz;

% Simulation parameters
dt = sim.dt;
maxTime = sim.max_time;
speedMultiplier = sim.speed_multiplier;

fprintf('  Grid: %dx%d | Mines: %.0f%% | Obstacles: %d\n', ...
    world.grid_rows, world.grid_cols, world.mine_density*100, world.num_obstacles);
fprintf('  Robot speed: %.1f m/s | Lidar range: %.1f m\n', ...
    robot.max_velocity, lidar.range);
fprintf('  Speed multiplier: %dx\n\n', speedMultiplier);

%% Step 1: Create World
fprintf('Creating world...\n');
gridRows = world.grid_rows;
gridCols = world.grid_cols;
cellSize = world.cell_size;
mineDensity = world.mine_density;
numObstacles = world.num_obstacles;

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

%% Step 4: Path Planning with Discovery
% Robot starts with NO knowledge of obstacles/mines
% As it explores, it discovers them and REPLANS the path

% Known obstacles/mines map (initially empty - robot doesn't know yet)
knownObstacles = false(gridRows, gridCols);
knownMines = false(gridRows, gridCols);

% Calculate INITIAL optimal path (assuming empty grid - no obstacles known yet)
fprintf('Computing initial optimal path (empty grid)...\n');
emptyMap = false(gridRows, gridCols);
initialPath = findShortestPath(emptyMap, startPos, endPos);
fprintf('Initial path length: %d steps (no obstacles known)\n', size(initialPath, 1));

% Generate coverage path using ACTUAL obstacle map (robot has sensors to detect obstacles)
% Robot will visit all accessible cells, avoiding real obstacles
path = generateCoveragePath(gridRows, gridCols, cellSize, obstacleMap, startPos);
waypointIdx = 1;
fprintf('Coverage path: %d waypoints (exploring all cells)\n\n', size(path,1));

%% Step 5: Initialize Robot
robotPos = [0.5, 0.5];  % Start position
robotHeading = 0;
robotVel = robot.max_velocity;

%% Step 6: Create DUAL Visualization (Original + SLAM Map)
fig = figure('Name', 'Minesweeper ROS2 - Original vs SLAM', 'NumberTitle', 'off', ...
            'Position', [50 50 1400 700], 'Color', [0.15 0.15 0.15]);

% ==================== LEFT PANEL: ORIGINAL WORLD MAP ====================
ax1 = subplot(1, 2, 1);
hold on;
axis equal;
xlim([0, gridCols*cellSize]);
ylim([0, gridRows*cellSize]);
set(ax1, 'Color', [0.2 0.2 0.2], 'XColor', 'w', 'YColor', 'w');
xlabel('X (m)', 'Color', 'w'); ylabel('Y (m)', 'Color', 'w');
title('ORIGINAL WORLD (Ground Truth)', 'Color', 'w', 'FontSize', 12);

% Draw grid lines
for i = 0:gridCols
    plot([i i]*cellSize, [0 gridRows]*cellSize, 'Color', [0.4 0.4 0.4]);
end
for i = 0:gridRows
    plot([0 gridCols]*cellSize, [i i]*cellSize, 'Color', [0.4 0.4 0.4]);
end

% OBSTACLES - Gray filled squares
[obsRows, obsCols] = find(obstacleMap);
for i = 1:length(obsRows)
    ox = (obsCols(i) - 1) * cellSize;
    oy = (obsRows(i) - 1) * cellSize;
    rectangle('Position', [ox+0.05, oy+0.05, cellSize-0.1, cellSize-0.1], ...
             'FaceColor', [0.5 0.5 0.5], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 2);
end

% MINES - Red X
[mineRows, mineCols] = find(mineMap);
mineWorldX = (mineCols - 0.5) * cellSize;
mineWorldY = (mineRows - 0.5) * cellSize;
plot(mineWorldX, mineWorldY, 'rx', 'MarkerSize', 15, 'LineWidth', 3);

% Robot on original map
hRobot1 = plot(robotPos(1), robotPos(2), 'bo', 'MarkerSize', 20, ...
             'MarkerFaceColor', [0.2 0.6 1], 'LineWidth', 2);
hTrail1 = plot(NaN, NaN, 'c-', 'LineWidth', 1.5);

% ==================== RIGHT PANEL: SLAM MAP (Robot's Knowledge) ====================
ax2 = subplot(1, 2, 2);
hold on;
axis equal;
xlim([0, gridCols*cellSize]);
ylim([0, gridRows*cellSize]);
set(ax2, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
xlabel('X (m)', 'Color', 'w'); ylabel('Y (m)', 'Color', 'w');
title('SLAM MAP (Lidar + Mine Detector)', 'Color', 'y', 'FontSize', 12);

% Draw grid lines (lighter for unexplored)
for i = 0:gridCols
    plot([i i]*cellSize, [0 gridRows]*cellSize, 'Color', [0.3 0.3 0.3]);
end
for i = 0:gridRows
    plot([0 gridCols]*cellSize, [i i]*cellSize, 'Color', [0.3 0.3 0.3]);
end

% Explored area (lidar scanned) - dim cells
hExploredArea = plot(NaN, NaN, 's', 'MarkerSize', 12, 'MarkerFaceColor', [0.15 0.2 0.15], ...
                    'MarkerEdgeColor', 'none');

% Lidar point cloud (blue dots like MathWorks image)
hLidarHits = plot(NaN, NaN, 'b.', 'MarkerSize', 4);  % Blue dots for obstacle edges

% Discovered obstacles (magenta outline like image)
hSlamObstacles = plot(NaN, NaN, 'm.', 'MarkerSize', 8);  % Magenta for known obstacles

% Discovered mines
hSlamMines = plot(NaN, NaN, 'go', 'MarkerSize', 30, 'LineWidth', 4);

% Robot on SLAM map
hRobot2 = plot(robotPos(1), robotPos(2), 'bo', 'MarkerSize', 20, ...
             'MarkerFaceColor', [0.2 0.6 1], 'LineWidth', 2);
hTrail2 = plot(NaN, NaN, 'c-', 'LineWidth', 1.5);

% Explored cells dots
hExplored = plot(NaN, NaN, 'g.', 'MarkerSize', 6);

% OPTIMAL PATH on SLAM map
if ~isempty(initialPath)
    initPathX = (initialPath(:,2) - 0.5) * cellSize;
    initPathY = (initialPath(:,1) - 0.5) * cellSize;
    hOptimalPath = plot(initPathX, initPathY, 'y-', 'LineWidth', 3);
else
    hOptimalPath = plot(NaN, NaN, 'y-', 'LineWidth', 3);
end

% Legend for SLAM map
legend([hRobot2, hSlamObstacles, hSlamMines, hOptimalPath], ...
       {'Robot', 'Known Obstacles', 'Detected Mines', 'Optimal Path'}, ...
       'Location', 'northeast', 'TextColor', 'w', 'Color', [0.2 0.2 0.2]);

% Store axes handles for use in loop
ax = ax2;  % Main plotting axis

% Status text
hStatus = uicontrol('Style', 'text', 'Position', [10 10 1380 40], ...
    'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'w', ...
    'FontSize', 12, 'HorizontalAlignment', 'left');

% Track explored cells
exploredCells = false(gridRows, gridCols);

% Accumulate ALL lidar point cloud hits 
allLidarPoints = [];  % Will store [x, y] of all lidar hits

fprintf('================================================\n');
fprintf('  STARTING SIMULATION - Dynamic Path Planning\n');
fprintf('  Yellow = Initial plan | Magenta = Current plan\n');
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
        
        % Calculate next position
        nextX = robotPos(1) + v * cos(robotHeading) * dt;
        nextY = robotPos(2) + v * sin(robotHeading) * dt;
        
        % Check if next position is in an obstacle
        nextGridR = max(1, min(gridRows, floor(nextY/cellSize) + 1));
        nextGridC = max(1, min(gridCols, floor(nextX/cellSize) + 1));
        
        % ===== LIDAR SIMULATION (params from config) =====
        obstacleDiscovered = false;
        mineDiscovered = false;
        
        lidarRange = lidar.range;       % From config
        lidarAngles = lidar.angles;     % From config
        numRays = length(lidarAngles);
        
        currentGridR = max(1, min(gridRows, floor(robotPos(2)/cellSize) + 1));
        currentGridC = max(1, min(gridCols, floor(robotPos(1)/cellSize) + 1));
        
        % ===== MINE DETECTOR - Only checks current cell =====
        if mineMap(currentGridR, currentGridC) && ~knownMines(currentGridR, currentGridC)
            knownMines(currentGridR, currentGridC) = true;
            mineDiscovered = true;
            markedMap(currentGridR, currentGridC) = true;
            minesFound = minesFound + 1;
            fprintf('[%.1fs] MINE DETECTOR: Mine at (%d,%d) - %d/%d\n', ...
                   time, currentGridR, currentGridC, minesFound, numMines);
        end
        
        % ===== LIDAR - Ray casting for point cloud =====
        % Cast rays in all directions and collect hit points
        lidarHits = [];  % Initialize empty array for this scan
        for rayIdx = 1:numRays
            angle = deg2rad(lidarAngles(rayIdx));
            
            % Ray march along this angle
            for dist = 0.1:0.15:lidarRange
                rayX = robotPos(1) + dist * cos(angle);
                rayY = robotPos(2) + dist * sin(angle);
                
                % Convert to grid
                rayGridR = max(1, min(gridRows, floor(rayY/cellSize) + 1));
                rayGridC = max(1, min(gridCols, floor(rayX/cellSize) + 1));
                
                % Mark cell as explored (lidar saw it)
                exploredCells(rayGridR, rayGridC) = true;
                
                % Check for obstacle hit - record point cloud
                if obstacleMap(rayGridR, rayGridC)
                    % Lidar hit obstacle - add to point cloud with noise
                    hitX = rayX + (rand()-0.5)*0.1;  % Add noise for realistic scatter
                    hitY = rayY + (rand()-0.5)*0.1;
                    lidarHits(end+1, :) = [hitX, hitY];
                    
                    if ~knownObstacles(rayGridR, rayGridC)
                        knownObstacles(rayGridR, rayGridC) = true;
                        obstacleDiscovered = true;
                    end
                    break;  % Ray stops at obstacle
                end
            end
        end
        
        if obstacleDiscovered
            fprintf('[%.1fs] LIDAR: New obstacles detected - REPLANNING!\n', time);
        end
        
        % Only move if NOT hitting an obstacle
        if ~obstacleMap(nextGridR, nextGridC)
            robotPos(1) = nextX;
            robotPos(2) = nextY;
        else
            % Hit obstacle - skip to next waypoint
            waypointIdx = waypointIdx + 1;
        end
        robotHeading = wrapToPi(robotHeading);
        
        % Check for mine
        gridR = max(1, min(gridRows, floor(robotPos(2)/cellSize) + 1));
        gridC = max(1, min(gridCols, floor(robotPos(1)/cellSize) + 1));
        
        mineAlert = false;
        if mineMap(gridR, gridC) && ~markedMap(gridR, gridC)
            mineAlert = true;
            detectedMap(gridR, gridC) = true;
            markedMap(gridR, gridC) = true;
            knownMines(gridR, gridC) = true;  % Add to known map
            minesFound = minesFound + 1;
            mineDiscovered = true;
            fprintf('[%.1fs] MINE at (%d,%d) - Total: %d/%d - REPLANNING!\n', ...
                   time, gridR, gridC, minesFound, numMines);
        end
        
        % REPLAN PATH if obstacle or mine discovered
        if obstacleDiscovered || mineDiscovered
            combinedKnown = knownObstacles | knownMines;
            newOptimalPath = findShortestPath(combinedKnown, startPos, endPos);
            
            % Update optimal path display (LIVE UPDATE)
            if ~isempty(newOptimalPath)
                newPathX = (newOptimalPath(:,2) - 0.5) * cellSize;
                newPathY = (newOptimalPath(:,1) - 0.5) * cellSize;
                set(hOptimalPath, 'XData', newPathX, 'YData', newPathY);
            else
                % No path exists - show message
                set(hOptimalPath, 'XData', NaN, 'YData', NaN);
            end
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
    
    % Update exploration tracking
    currentGridR = max(1, min(gridRows, floor(robotPos(2)/cellSize) + 1));
    currentGridC = max(1, min(gridCols, floor(robotPos(1)/cellSize) + 1));
    exploredCells(currentGridR, currentGridC) = true;
    
    % ===== UPDATE ORIGINAL WORLD MAP (Left Panel) =====
    set(hRobot1, 'XData', robotPos(1), 'YData', robotPos(2));
    set(hTrail1, 'XData', trail(:,1), 'YData', trail(:,2));
    
    % ===== UPDATE SLAM MAP (Right Panel) =====
    set(hRobot2, 'XData', robotPos(1), 'YData', robotPos(2));
    set(hTrail2, 'XData', trail(:,1), 'YData', trail(:,2));
    
    % Accumulate lidar point cloud hits (like MathWorks image)
    if ~isempty(lidarHits)
        allLidarPoints = [allLidarPoints; lidarHits];
    end
    
    % Show lidar point cloud (blue dots showing obstacle edges)
    if ~isempty(allLidarPoints)
        set(hLidarHits, 'XData', allLidarPoints(:,1), 'YData', allLidarPoints(:,2));
    end
    
    % Show explored area on SLAM map (cells lidar has seen - dim)
    [expR, expC] = find(exploredCells & ~knownObstacles);
    if ~isempty(expR)
        expX = (expC - 0.5) * cellSize;
        expY = (expR - 0.5) * cellSize;
        set(hExplored, 'XData', expX, 'YData', expY);
    end
    
    % Show discovered obstacles on SLAM map (from point cloud)
    [obsR, obsC] = find(knownObstacles);
    if ~isempty(obsR)
        obsX = (obsC - 0.5) * cellSize;
        obsY = (obsR - 0.5) * cellSize;
        set(hSlamObstacles, 'XData', obsX, 'YData', obsY);
    end
    
    % Show detected mines on SLAM map (green circles)
    [mineR, mineC] = find(knownMines);
    if ~isempty(mineR)
        mineX = (mineC - 0.5) * cellSize;
        mineY = (mineR - 0.5) * cellSize;
        set(hSlamMines, 'XData', mineX, 'YData', mineY);
    end
    
    % Show explored cells on SLAM map
    [expR, expC] = find(exploredCells);
    if ~isempty(expR)
        expX = (expC - 0.5) * cellSize;
        expY = (expR - 0.5) * cellSize;
        set(hExplored, 'XData', expX, 'YData', expY);
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

function path = generateCoveragePath(rows, cols, cellSize, obstacleMap, startPos)
    % Generate a path that visits ALL accessible cells using A* pathfinding
    % Uses A* to navigate between unvisited cells
    
    visited = false(rows, cols);
    path = [];
    
    % Mark obstacles as visited (can't go there)
    visited = visited | obstacleMap;
    
    % Start position
    currentCell = startPos;
    visited(currentCell(1), currentCell(2)) = true;
    path(end+1,:) = [(currentCell(2)-0.5)*cellSize, (currentCell(1)-0.5)*cellSize];
    
    % Keep exploring until all cells visited
    while true
        % Find nearest unvisited cell using A*
        [nearestCell, pathToCell] = findNearestUnvisited(currentCell, visited, obstacleMap, rows, cols);
        
        if isempty(nearestCell)
            break;  % All cells visited
        end
        
        % Add A* path to the exploration path
        for i = 2:size(pathToCell, 1)
            cell = pathToCell(i,:);
            path(end+1,:) = [(cell(2)-0.5)*cellSize, (cell(1)-0.5)*cellSize];
            visited(cell(1), cell(2)) = true;
        end
        
        currentCell = nearestCell;
    end
end

function [nearest, pathTo] = findNearestUnvisited(start, visited, obstacleMap, rows, cols)
    % A* search to find path to nearest unvisited cell
    
    nearest = [];
    pathTo = [];
    
    % Check if all visited
    if all(visited(:))
        return;
    end
    
    % A* setup
    openList = start;
    gScore = inf(rows, cols);
    gScore(start(1), start(2)) = 0;
    cameFrom = zeros(rows, cols, 2);
    
    directions = [-1 0; 1 0; 0 -1; 0 1];  % 4 directions
    
    while ~isempty(openList)
        % Get node with lowest gScore (BFS-like for nearest)
        [~, idx] = min(arrayfun(@(i) gScore(openList(i,1), openList(i,2)), 1:size(openList,1)));
        current = openList(idx,:);
        openList(idx,:) = [];
        
        % Check if this is an unvisited cell (not start)
        if ~isequal(current, start) && ~visited(current(1), current(2))
            nearest = current;
            pathTo = reconstructAStarPath(cameFrom, current, start);
            return;
        end
        
        % Explore neighbors
        for d = 1:4
            neighbor = current + directions(d,:);
            
            if neighbor(1) >= 1 && neighbor(1) <= rows && ...
               neighbor(2) >= 1 && neighbor(2) <= cols && ...
               ~obstacleMap(neighbor(1), neighbor(2))
                
                tentativeG = gScore(current(1), current(2)) + 1;
                
                if tentativeG < gScore(neighbor(1), neighbor(2))
                    cameFrom(neighbor(1), neighbor(2), :) = current;
                    gScore(neighbor(1), neighbor(2)) = tentativeG;
                    
                    if ~any(all(openList == neighbor, 2))
                        openList(end+1,:) = neighbor;
                    end
                end
            end
        end
    end
end

function path = reconstructAStarPath(cameFrom, current, start)
    path = current;
    while ~isequal(current, start)
        prev = squeeze(cameFrom(current(1), current(2), :))';
        if all(prev == 0)
            break;
        end
        path = [prev; path];
        current = prev;
    end
end
