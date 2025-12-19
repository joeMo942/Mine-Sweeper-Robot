classdef PathPlanner < handle
    %PATHPLANNER Path planning for minesweeper robot
    %   Generates coverage paths and handles obstacle avoidance
    
    properties
        gridSize        % [rows, cols] of the grid
        cellSize        % Size of each cell in meters
        algorithm       % Planning algorithm: 'boustrophedon', 'spiral'
        safetyMargin    % Safety margin around obstacles
        
        % Path data
        waypoints       % [N x 2] array of waypoints in world coordinates
        currentIndex    % Current waypoint index
        
        % Maps for planning
        visitedMap      % Logical map of visited cells
        obstacleMap     % Copy of obstacle map
        mineMap         % Copy of marked mine map
    end
    
    methods
        function obj = PathPlanner(gridSize, cellSize, algorithm)
            %PATHPLANNER Construct path planner
            %   obj = PathPlanner(gridSize) - default boustrophedon
            %   obj = PathPlanner(gridSize, cellSize, algorithm)
            
            obj.gridSize = gridSize;
            
            if nargin < 2
                obj.cellSize = 1.0;
            else
                obj.cellSize = cellSize;
            end
            
            if nargin < 3
                obj.algorithm = 'boustrophedon';
            else
                obj.algorithm = algorithm;
            end
            
            obj.safetyMargin = 0.3;
            obj.waypoints = [];
            obj.currentIndex = 1;
            obj.visitedMap = false(gridSize);
            obj.obstacleMap = false(gridSize);
            obj.mineMap = false(gridSize);
        end
        
        function path = planPath(obj, startPos, field)
            %PLANPATH Generate coverage path from start position
            %   path = planPath(obj, startPos)
            %   path = planPath(obj, startPos, field) - considers obstacles
            
            if nargin >= 3
                obj.obstacleMap = field.obstacleMap;
                obj.mineMap = field.markedMap;
            end
            
            switch obj.algorithm
                case 'boustrophedon'
                    path = obj.planBoustrophedon(startPos);
                case 'spiral'
                    path = obj.planSpiral(startPos);
                otherwise
                    path = obj.planBoustrophedon(startPos);
            end
            
            obj.waypoints = path;
            obj.currentIndex = 1;
        end
        
        function path = planBoustrophedon(obj, startPos)
            %PLANBOUSTROPHEDON Generate boustrophedon (lawn mower) path
            
            path = [];
            rows = obj.gridSize(1);
            cols = obj.gridSize(2);
            
            % Determine starting corner based on start position
            startGrid = floor(startPos / obj.cellSize) + 1;
            startGrid = max([1, 1], min(obj.gridSize, startGrid));
            
            % Generate path
            direction = 1; % 1 = right, -1 = left
            
            for row = 1:rows
                if direction == 1
                    colRange = 1:cols;
                else
                    colRange = cols:-1:1;
                end
                
                for col = colRange
                    % Skip obstacles
                    if obj.obstacleMap(row, col)
                        continue;
                    end
                    
                    % Add waypoint at cell center [x, y] format
                    % x from col, y from row
                    worldX = (col - 0.5) * obj.cellSize;
                    worldY = (row - 0.5) * obj.cellSize;
                    path(end+1, :) = [worldX, worldY];
                end
                
                % Alternate direction
                direction = -direction;
            end
        end
        
        function path = planSpiral(obj, startPos)
            %PLANSPIRAL Generate inward spiral path
            
            path = [];
            rows = obj.gridSize(1);
            cols = obj.gridSize(2);
            
            visited = false(rows, cols);
            
            % Direction vectors: right, down, left, up
            dr = [0, 1, 0, -1];
            dc = [1, 0, -1, 0];
            
            r = 1;
            c = 1;
            dir = 1; % Start going right
            
            for step = 1:(rows * cols)
                if ~visited(r, c) && ~obj.obstacleMap(r, c)
                    visited(r, c) = true;
                    % [x, y] format: x from col, y from row
                    worldX = (c - 0.5) * obj.cellSize;
                    worldY = (r - 0.5) * obj.cellSize;
                    path(end+1, :) = [worldX, worldY];
                end
                
                % Try to continue in current direction
                nextR = r + dr(dir);
                nextC = c + dc(dir);
                
                % Check if we need to turn
                if nextR < 1 || nextR > rows || nextC < 1 || nextC > cols || visited(nextR, nextC)
                    % Turn right
                    dir = mod(dir, 4) + 1;
                    nextR = r + dr(dir);
                    nextC = c + dc(dir);
                    
                    % If still blocked, try other directions
                    attempts = 0;
                    while (nextR < 1 || nextR > rows || nextC < 1 || nextC > cols || visited(nextR, nextC)) && attempts < 4
                        dir = mod(dir, 4) + 1;
                        nextR = r + dr(dir);
                        nextC = c + dc(dir);
                        attempts = attempts + 1;
                    end
                    
                    if attempts >= 4
                        break; % No more moves possible
                    end
                end
                
                r = nextR;
                c = nextC;
            end
        end
        
        function waypoint = getNextWaypoint(obj)
            %GETNEXTWAYPOINT Get the next waypoint in the path
            
            if isempty(obj.waypoints) || obj.currentIndex > size(obj.waypoints, 1)
                waypoint = [];
                return;
            end
            
            waypoint = obj.waypoints(obj.currentIndex, :);
        end
        
        function advanceWaypoint(obj)
            %ADVANCEWAYPOINT Move to the next waypoint
            obj.currentIndex = obj.currentIndex + 1;
        end
        
        function complete = isPathComplete(obj)
            %ISPATHCOMPLETE Check if path is complete
            complete = obj.currentIndex > size(obj.waypoints, 1);
        end
        
        function progress = getProgress(obj)
            %GETPROGRESS Get path progress (0 to 1)
            if isempty(obj.waypoints)
                progress = 1;
            else
                progress = (obj.currentIndex - 1) / size(obj.waypoints, 1);
            end
        end
        
        function replan(obj, currentPos, field)
            %REPLAN Replan path from current position
            
            % Update maps
            if nargin >= 3
                obj.obstacleMap = field.obstacleMap;
                obj.mineMap = field.markedMap;
            end
            
            % Mark visited cells
            if ~isempty(obj.waypoints) && obj.currentIndex > 1
                for i = 1:obj.currentIndex-1
                    % waypoints are [x, y], convert to [row, col]
                    wp = obj.waypoints(i, :);
                    col = floor(wp(1) / obj.cellSize) + 1;
                    row = floor(wp(2) / obj.cellSize) + 1;
                    row = max(1, min(obj.gridSize(1), row));
                    col = max(1, min(obj.gridSize(2), col));
                    obj.visitedMap(row, col) = true;
                end
            end
            
            % Generate new path for unvisited cells
            obj.planPath(currentPos, field);
        end
        
        function visualizePath(obj, ax)
            %VISUALIZEPATH Visualize the planned path
            
            if isempty(obj.waypoints)
                return;
            end
            
            if nargin < 2
                ax = gca;
            end
            
            hold(ax, 'on');
            
            % Plot full path - waypoints are now [x, y]
            plot(ax, obj.waypoints(:,1), obj.waypoints(:,2), 'b--', 'LineWidth', 1);
            
            % Highlight current waypoint
            if obj.currentIndex <= size(obj.waypoints, 1)
                current = obj.waypoints(obj.currentIndex, :);
                plot(ax, current(1), current(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
            end
            
            % Mark start and end
            plot(ax, obj.waypoints(1,1), obj.waypoints(1,2), 'g^', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
            plot(ax, obj.waypoints(end,1), obj.waypoints(end,2), 'rv', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
            
            hold(ax, 'off');
        end
        
        function n = getNumWaypoints(obj)
            %GETNUMWAYPOINTS Get total number of waypoints
            n = size(obj.waypoints, 1);
        end
        
        function remaining = getRemainingWaypoints(obj)
            %GETREMAININGWAYPOINTS Get number of remaining waypoints
            remaining = max(0, size(obj.waypoints, 1) - obj.currentIndex + 1);
        end
    end
end
