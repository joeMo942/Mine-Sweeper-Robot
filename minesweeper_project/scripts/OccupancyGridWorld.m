classdef OccupancyGridWorld < handle
    %OCCUPANCYGRIDWORLD Step 1: Occupancy Grid World for Minesweeper Robot
    %   Creates and manages the occupancy map with mines and obstacles
    
    properties
        map                 % occupancyMap object
        mapWidth            % Width in meters
        mapHeight           % Height in meters
        resolution          % Cells per meter
        
        % Locations
        mineLocations       % [N x 2] array of mine positions [x, y]
        obstacleLocations   % [M x 2] array of obstacle positions [x, y]
        hazardZones         % [K x 3] array of hazard zones [x, y, radius]
        
        % Configuration
        mineRadius          % Radius to mark around mines
        numMines            % Number of mines
        numObstacles        % Number of obstacles
        
        % Start and goal
        startPosition       % [x, y] start position
        goalPosition        % [x, y] goal position (optional)
    end
    
    properties (Constant)
        % Occupancy values
        FREE = 0
        OCCUPIED = 1
        UNKNOWN = 0.5
        MINE = 0.9          % High probability = mine
        OBSTACLE = 0.8      % Obstacle
    end
    
    methods
        function obj = OccupancyGridWorld(width, height, resolution)
            %OCCUPANCYGRIDWORLD Create occupancy grid world
            %   obj = OccupancyGridWorld(20, 20, 10)  % 20x20m, 10 cells/m
            
            if nargin < 1, width = 20; end
            if nargin < 2, height = 20; end
            if nargin < 3, resolution = 10; end
            
            obj.mapWidth = width;
            obj.mapHeight = height;
            obj.resolution = resolution;
            
            % Create occupancy map
            obj.map = occupancyMap(width, height, resolution);
            
            % Initialize properties
            obj.mineLocations = [];
            obj.obstacleLocations = [];
            obj.hazardZones = [];
            obj.mineRadius = 0.5;  % 0.5m radius around mines
            obj.numMines = 15;
            obj.numObstacles = 5;
            obj.startPosition = [1, 1];
            obj.goalPosition = [width-1, height-1];
            
            fprintf('[OccupancyGridWorld] Created %dx%d map with resolution %d\n', ...
                    width, height, resolution);
        end
        
        function generateWorld(obj, numMines, numObstacles, startPos)
            %GENERATEWORLD Generate random world with mines and obstacles
            
            if nargin >= 2, obj.numMines = numMines; end
            if nargin >= 3, obj.numObstacles = numObstacles; end
            if nargin >= 4, obj.startPosition = startPos; end
            
            fprintf('[OccupancyGridWorld] Generating world...\n');
            
            % Clear previous
            obj.mineLocations = [];
            obj.obstacleLocations = [];
            obj.hazardZones = [];
            
            % Reset map to free space
            setOccupancy(obj.map, ones(obj.map.GridSize) * obj.FREE);
            
            % Add boundary walls
            obj.addBoundaryWalls();
            
            % Generate random mine positions (avoid start area)
            safeRadius = 2;  % 2m safe zone around start
            attempts = 0;
            while size(obj.mineLocations, 1) < obj.numMines && attempts < 1000
                x = rand() * (obj.mapWidth - 2) + 1;
                y = rand() * (obj.mapHeight - 2) + 1;
                
                % Check not too close to start
                if norm([x, y] - obj.startPosition) > safeRadius
                    obj.mineLocations(end+1, :) = [x, y];
                end
                attempts = attempts + 1;
            end
            
            % Generate random obstacles
            attempts = 0;
            while size(obj.obstacleLocations, 1) < obj.numObstacles && attempts < 1000
                x = rand() * (obj.mapWidth - 4) + 2;
                y = rand() * (obj.mapHeight - 4) + 2;
                
                % Check not too close to start or mines
                tooClose = false;
                if norm([x, y] - obj.startPosition) < safeRadius
                    tooClose = true;
                end
                for i = 1:size(obj.mineLocations, 1)
                    if norm([x, y] - obj.mineLocations(i,:)) < 1
                        tooClose = true;
                        break;
                    end
                end
                
                if ~tooClose
                    obj.obstacleLocations(end+1, :) = [x, y];
                    % Add obstacle to map
                    obj.addObstacle(x, y, 0.5);  % 0.5m radius obstacle
                end
                attempts = attempts + 1;
            end
            
            fprintf('[OccupancyGridWorld] Placed %d mines and %d obstacles\n', ...
                    size(obj.mineLocations, 1), size(obj.obstacleLocations, 1));
        end
        
        function addBoundaryWalls(obj)
            %ADDBOUNDARYWALLS Add walls around the map edges
            
            % Wall thickness
            wallThickness = 0.1;
            
            % Create wall coordinates
            % Bottom wall
            x = linspace(0, obj.mapWidth, obj.mapWidth * obj.resolution);
            y = zeros(size(x));
            setOccupancy(obj.map, [x', y'], ones(length(x), 1));
            
            % Top wall
            y = ones(size(x)) * obj.mapHeight;
            setOccupancy(obj.map, [x', y'], ones(length(x), 1));
            
            % Left wall
            y = linspace(0, obj.mapHeight, obj.mapHeight * obj.resolution);
            x = zeros(size(y));
            setOccupancy(obj.map, [x', y'], ones(length(y), 1));
            
            % Right wall
            x = ones(size(y)) * obj.mapWidth;
            setOccupancy(obj.map, [x', y'], ones(length(y), 1));
        end
        
        function addObstacle(obj, x, y, radius)
            %ADDOBSTACLE Add circular obstacle to map
            
            % Generate points in circle
            theta = linspace(0, 2*pi, 36);
            for r = 0:0.05:radius
                px = x + r * cos(theta);
                py = y + r * sin(theta);
                setOccupancy(obj.map, [px', py'], ones(length(theta), 1) * obj.OBSTACLE);
            end
        end
        
        function markHazard(obj, x, y, radius)
            %MARKHAZARD Mark a hazard zone (detected mine)
            %   This is called when a mine is detected
            
            if nargin < 4, radius = obj.mineRadius; end
            
            % Add to hazard zones
            obj.hazardZones(end+1, :) = [x, y, radius];
            
            % Mark cells as occupied in map
            theta = linspace(0, 2*pi, 36);
            for r = 0:0.05:radius
                px = x + r * cos(theta);
                py = y + r * sin(theta);
                setOccupancy(obj.map, [px', py'], ones(length(theta), 1) * obj.MINE);
            end
            
            fprintf('[OccupancyGridWorld] Marked hazard at (%.2f, %.2f) radius %.2f\n', ...
                    x, y, radius);
        end
        
        function detected = checkMineProximity(obj, robotPos, sensorRange)
            %CHECKMINEPROXIMITY Check if robot is near a mine
            
            detected = false;
            
            for i = 1:size(obj.mineLocations, 1)
                dist = norm(robotPos - obj.mineLocations(i,:));
                if dist <= sensorRange
                    detected = true;
                    return;
                end
            end
        end
        
        function [minePos, dist] = getNearestMine(obj, robotPos)
            %GETNEARESTMINE Get position of nearest mine
            
            if isempty(obj.mineLocations)
                minePos = [];
                dist = inf;
                return;
            end
            
            distances = vecnorm(obj.mineLocations - robotPos, 2, 2);
            [dist, idx] = min(distances);
            minePos = obj.mineLocations(idx, :);
        end
        
        function occGrid = getOccupancyGrid(obj)
            %GETOCCUPANCYGRID Get the occupancy grid matrix
            occGrid = occupancyMatrix(obj.map);
        end
        
        function msg = getOccupancyGridMsg(obj)
            %GETOCCUPANCYGRIDMSG Get as ROS2 OccupancyGrid message
            
            msg = ros2message('nav_msgs/OccupancyGrid');
            
            % Header
            msg.header.stamp = ros2time(ros2node('/temp'), 'now');
            msg.header.frame_id = 'map';
            
            % Map metadata
            msg.info.resolution = 1 / obj.resolution;
            msg.info.width = uint32(obj.map.GridSize(2));
            msg.info.height = uint32(obj.map.GridSize(1));
            msg.info.origin.position.x = 0;
            msg.info.origin.position.y = 0;
            msg.info.origin.position.z = 0;
            
            % Occupancy data (convert to int8)
            occMatrix = occupancyMatrix(obj.map);
            msg.data = int8(occMatrix(:) * 100);  % 0-100 scale
        end
        
        function show(obj, varargin)
            %SHOW Display the occupancy map
            
            figure('Name', 'Minesweeper Occupancy Grid', 'NumberTitle', 'off');
            show(obj.map);
            hold on;
            
            % Plot mines (hidden - shown as X when detected)
            if ~isempty(obj.mineLocations)
                plot(obj.mineLocations(:,1), obj.mineLocations(:,2), ...
                    'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Mines');
            end
            
            % Plot obstacles
            if ~isempty(obj.obstacleLocations)
                plot(obj.obstacleLocations(:,1), obj.obstacleLocations(:,2), ...
                    'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'DisplayName', 'Obstacles');
            end
            
            % Plot hazard zones
            if ~isempty(obj.hazardZones)
                theta = linspace(0, 2*pi, 50);
                for i = 1:size(obj.hazardZones, 1)
                    x = obj.hazardZones(i,1) + obj.hazardZones(i,3) * cos(theta);
                    y = obj.hazardZones(i,2) + obj.hazardZones(i,3) * sin(theta);
                    plot(x, y, 'r-', 'LineWidth', 2);
                    fill(x, y, 'r', 'FaceAlpha', 0.3);
                end
            end
            
            % Plot start and goal
            plot(obj.startPosition(1), obj.startPosition(2), 'go', ...
                'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
            plot(obj.goalPosition(1), obj.goalPosition(2), 'bp', ...
                'MarkerSize', 15, 'MarkerFaceColor', 'b', 'DisplayName', 'Goal');
            
            legend('Location', 'bestoutside');
            title('Minesweeper Occupancy Grid World');
            xlabel('X (m)');
            ylabel('Y (m)');
            hold off;
        end
        
        function stats = getStats(obj)
            %GETSTATS Get world statistics
            
            stats.mapSize = [obj.mapWidth, obj.mapHeight];
            stats.resolution = obj.resolution;
            stats.numMines = size(obj.mineLocations, 1);
            stats.numObstacles = size(obj.obstacleLocations, 1);
            stats.numHazardsMarked = size(obj.hazardZones, 1);
            stats.minesRemaining = stats.numMines - stats.numHazardsMarked;
        end
    end
end
