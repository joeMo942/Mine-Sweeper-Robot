classdef SpanningTreeCoverage
    % SpanningTreeCoverage - Optimized coverage path planning
    %
    % Provides coverage algorithms that generate ADJACENT waypoints
    % so the robot can travel between them without hitting obstacles.
    %
    % Usage:
    %   path = SpanningTreeCoverage.generateSTCPath(rows, cols, cellSize, obstacleMap, startPos);
    
    properties
        rows        % Number of rows in grid
        cols        % Number of columns in grid
        cellSize    % Size of each cell in meters
        obstacleMap % Boolean map of obstacles
    end
    
    methods
        function obj = SpanningTreeCoverage(rows, cols, cellSize, obstacleMap)
            obj.rows = rows;
            obj.cols = cols;
            obj.cellSize = cellSize;
            obj.obstacleMap = obstacleMap;
        end
        
        function path = generateConnectedBoustrophedonPath(obj, startPos)
            % Generate Boustrophedon path with A* connections between rows
            % Ensures ALL consecutive waypoints are reachable
            %
            % This is the KEY FIX: we use A* to find paths between
            % non-adjacent cells when switching rows or avoiding obstacles
            
            visited = false(obj.rows, obj.cols);
            visited = visited | obj.obstacleMap;
            
            % Generate the order of cells to visit (Boustrophedon)
            cellsToVisit = [];
            direction = 1;
            
            for r = 1:obj.rows
                if direction == 1
                    colRange = 1:obj.cols;
                else
                    colRange = obj.cols:-1:1;
                end
                
                for c = colRange
                    if ~obj.obstacleMap(r, c)
                        cellsToVisit(end+1, :) = [r, c];
                    end
                end
                
                direction = -direction;
            end
            
            if isempty(cellsToVisit)
                path = [];
                return;
            end
            
            % Now create actual path with A* connections
            pathCells = cellsToVisit(1, :);
            
            for i = 2:size(cellsToVisit, 1)
                currentCell = pathCells(end, :);
                nextCell = cellsToVisit(i, :);
                
                % Check if cells are adjacent
                if obj.areAdjacent(currentCell, nextCell)
                    pathCells(end+1, :) = nextCell;
                else
                    % Use A* to find path between cells
                    astarPath = obj.findPath(currentCell, nextCell);
                    if ~isempty(astarPath)
                        % Add all cells except the first (current cell)
                        pathCells = [pathCells; astarPath(2:end, :)];
                    else
                        % No path exists - skip this cell
                        fprintf('Warning: Cannot reach cell (%d,%d), skipping\n', nextCell(1), nextCell(2));
                    end
                end
            end
            
            % Convert to world coordinates
            path = zeros(size(pathCells, 1), 2);
            for i = 1:size(pathCells, 1)
                path(i, :) = [(pathCells(i,2) - 0.5) * obj.cellSize, ...
                              (pathCells(i,1) - 0.5) * obj.cellSize];
            end
            
            fprintf('Connected Boustrophedon: %d waypoints (all adjacent)\n', size(path, 1));
        end
        
        function adjacent = areAdjacent(~, cell1, cell2)
            % Check if two cells are adjacent (4-connectivity)
            diff = abs(cell1 - cell2);
            adjacent = (diff(1) + diff(2)) == 1;
        end
        
        function path = findPath(obj, startCell, endCell)
            % A* pathfinding between two cells
            
            if obj.obstacleMap(startCell(1), startCell(2)) || ...
               obj.obstacleMap(endCell(1), endCell(2))
                path = [];
                return;
            end
            
            % A* implementation
            openList = startCell;
            gScore = inf(obj.rows, obj.cols);
            gScore(startCell(1), startCell(2)) = 0;
            fScore = inf(obj.rows, obj.cols);
            fScore(startCell(1), startCell(2)) = obj.heuristic(startCell, endCell);
            cameFrom = zeros(obj.rows, obj.cols, 2);
            
            directions = [-1 0; 1 0; 0 -1; 0 1];
            
            while ~isempty(openList)
                % Find node with lowest fScore
                [~, idx] = min(arrayfun(@(i) fScore(openList(i,1), openList(i,2)), 1:size(openList,1)));
                current = openList(idx, :);
                
                if isequal(current, endCell)
                    path = obj.reconstructPath(cameFrom, current);
                    return;
                end
                
                openList(idx, :) = [];
                
                for d = 1:4
                    neighbor = current + directions(d, :);
                    
                    if neighbor(1) >= 1 && neighbor(1) <= obj.rows && ...
                       neighbor(2) >= 1 && neighbor(2) <= obj.cols && ...
                       ~obj.obstacleMap(neighbor(1), neighbor(2))
                        
                        tentativeG = gScore(current(1), current(2)) + 1;
                        
                        if tentativeG < gScore(neighbor(1), neighbor(2))
                            cameFrom(neighbor(1), neighbor(2), :) = current;
                            gScore(neighbor(1), neighbor(2)) = tentativeG;
                            fScore(neighbor(1), neighbor(2)) = tentativeG + obj.heuristic(neighbor, endCell);
                            
                            if ~any(all(openList == neighbor, 2))
                                openList(end+1, :) = neighbor;
                            end
                        end
                    end
                end
            end
            
            path = [];  % No path found
        end
        
        function h = heuristic(~, pos, goal)
            % Manhattan distance heuristic
            h = abs(pos(1) - goal(1)) + abs(pos(2) - goal(2));
        end
        
        function path = reconstructPath(~, cameFrom, current)
            path = current;
            while any(cameFrom(current(1), current(2), :))
                current = squeeze(cameFrom(current(1), current(2), :))';
                if all(current == 0)
                    break;
                end
                path = [current; path];
            end
        end
        
        function valid = isValidCell(obj, pos)
            valid = pos(1) >= 1 && pos(1) <= obj.rows && ...
                    pos(2) >= 1 && pos(2) <= obj.cols && ...
                    ~obj.obstacleMap(pos(1), pos(2));
        end
    end
    
    methods (Static)
        function path = generateSTCPath(rows, cols, cellSize, obstacleMap, startPos)
            % Static convenience method - uses connected Boustrophedon
            %
            % This generates a path where ALL consecutive waypoints are
            % adjacent, so the robot can reach each one without teleporting
            
            stc = SpanningTreeCoverage(rows, cols, cellSize, obstacleMap);
            path = stc.generateConnectedBoustrophedonPath(startPos);
        end
    end
end
