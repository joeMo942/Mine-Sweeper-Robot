classdef MinefieldGenerator < handle
    %MINEFIELDGENERATOR Generates and manages the minefield environment
    %   Creates a grid-based minefield with mines and obstacles
    
    properties
        gridRows        % Number of rows in the grid
        gridCols        % Number of columns in the grid
        cellSize        % Size of each cell in meters
        mineDensity     % Density of mines (0-1)
        numObstacles    % Number of obstacles
        
        % Field data
        mineMap         % Logical matrix of mine locations
        obstacleMap     % Logical matrix of obstacle locations
        revealedMap     % Logical matrix of revealed cells
        markedMap       % Logical matrix of marked mines
        
        % Mine and obstacle locations (for quick access)
        mineLocations   % [N x 2] array of mine positions
        obstacleLocations % [M x 2] array of obstacle positions
    end
    
    methods
        function obj = MinefieldGenerator(rows, cols, density, varargin)
            %MINEFIELDGENERATOR Construct minefield generator
            %   obj = MinefieldGenerator(rows, cols, density)
            %   obj = MinefieldGenerator(rows, cols, density, numObstacles)
            
            obj.gridRows = rows;
            obj.gridCols = cols;
            obj.mineDensity = density;
            obj.cellSize = 1.0; % Default cell size
            
            if nargin >= 4
                obj.numObstacles = varargin{1};
            else
                obj.numObstacles = 5;
            end
            
            % Initialize maps
            obj.mineMap = false(rows, cols);
            obj.obstacleMap = false(rows, cols);
            obj.revealedMap = false(rows, cols);
            obj.markedMap = false(rows, cols);
            obj.mineLocations = [];
            obj.obstacleLocations = [];
        end
        
        function generateField(obj, startPos)
            %GENERATEFIELD Generate random minefield
            %   generateField(obj) generates with random start
            %   generateField(obj, startPos) keeps startPos area clear
            
            if nargin < 2
                startPos = [1, 1];
            end
            
            fprintf('Generating minefield (%dx%d)...\n', obj.gridRows, obj.gridCols);
            
            % Reset maps
            obj.mineMap = false(obj.gridRows, obj.gridCols);
            obj.obstacleMap = false(obj.gridRows, obj.gridCols);
            obj.revealedMap = false(obj.gridRows, obj.gridCols);
            obj.markedMap = false(obj.gridRows, obj.gridCols);
            
            % Calculate number of mines
            totalCells = obj.gridRows * obj.gridCols;
            numMines = round(totalCells * obj.mineDensity);
            
            % Create safe zone around start position (3x3 area)
            safeRows = max(1, startPos(1)-1):min(obj.gridRows, startPos(1)+1);
            safeCols = max(1, startPos(2)-1):min(obj.gridCols, startPos(2)+1);
            safeZone = false(obj.gridRows, obj.gridCols);
            safeZone(safeRows, safeCols) = true;
            
            % Generate mine positions (excluding safe zone)
            availableCells = find(~safeZone);
            if length(availableCells) < numMines
                numMines = length(availableCells);
            end
            
            mineIndices = availableCells(randperm(length(availableCells), numMines));
            obj.mineMap(mineIndices) = true;
            
            % Store mine locations as [row, col] pairs
            [mineRows, mineCols] = find(obj.mineMap);
            obj.mineLocations = [mineRows, mineCols];
            
            % Generate obstacles (avoiding mines and safe zone)
            availableForObstacles = find(~obj.mineMap & ~safeZone);
            numObs = min(obj.numObstacles, length(availableForObstacles));
            
            if numObs > 0
                obsIndices = availableForObstacles(randperm(length(availableForObstacles), numObs));
                obj.obstacleMap(obsIndices) = true;
                
                [obsRows, obsCols] = find(obj.obstacleMap);
                obj.obstacleLocations = [obsRows, obsCols];
            end
            
            fprintf('  Mines placed: %d\n', numMines);
            fprintf('  Obstacles placed: %d\n', numObs);
            fprintf('Minefield generation complete.\n\n');
        end
        
        function [hasMine, neighbors] = checkCell(obj, row, col)
            %CHECKCELL Check if a cell contains a mine
            %   [hasMine, neighbors] = checkCell(obj, row, col)
            
            if row < 1 || row > obj.gridRows || col < 1 || col > obj.gridCols
                hasMine = false;
                neighbors = -1;
                return;
            end
            
            hasMine = obj.mineMap(row, col);
            
            % Count neighboring mines
            neighbors = 0;
            for dr = -1:1
                for dc = -1:1
                    if dr == 0 && dc == 0
                        continue;
                    end
                    nr = row + dr;
                    nc = col + dc;
                    if nr >= 1 && nr <= obj.gridRows && nc >= 1 && nc <= obj.gridCols
                        if obj.mineMap(nr, nc)
                            neighbors = neighbors + 1;
                        end
                    end
                end
            end
            
            obj.revealedMap(row, col) = true;
        end
        
        function success = markMine(obj, row, col)
            %MARKMINE Mark a cell as containing a mine
            %   success = markMine(obj, row, col)
            
            if row < 1 || row > obj.gridRows || col < 1 || col > obj.gridCols
                success = false;
                return;
            end
            
            obj.markedMap(row, col) = true;
            success = true;
        end
        
        function worldPos = gridToWorld(obj, gridPos)
            %GRIDTOWORLD Convert grid coordinates to world coordinates
            %   worldPos = gridToWorld(obj, gridPos)
            %   gridPos: [row, col] or [N x 2] array
            %   worldPos: [x, y] in meters (x from col, y from row)
            
            if size(gridPos, 1) == 1
                % Single position: swap row,col to x,y
                worldPos = ([gridPos(2), gridPos(1)] - 0.5) * obj.cellSize;
            else
                % Multiple positions: swap columns
                worldPos = ([gridPos(:,2), gridPos(:,1)] - 0.5) * obj.cellSize;
            end
        end
        
        function gridPos = worldToGrid(obj, worldPos)
            %WORLDTOGRID Convert world coordinates to grid coordinates
            %   gridPos = worldToGrid(obj, worldPos)
            %   worldPos: [x, y] in meters
            %   gridPos: [row, col] (row from y, col from x)
            
            % x -> col, y -> row
            col = floor(worldPos(1) / obj.cellSize) + 1;
            row = floor(worldPos(2) / obj.cellSize) + 1;
            gridPos = [row, col];
            gridPos = max([1, 1], min([obj.gridRows, obj.gridCols], gridPos));
        end
        
        function mines = getMineLocations(obj)
            %GETMINELOCATIONS Get all mine locations in world coordinates
            mines = obj.gridToWorld(obj.mineLocations);
        end
        
        function obstacles = getObstacleLocations(obj)
            %GETOBSTACLELOCATIONS Get all obstacle locations in world coordinates
            obstacles = obj.gridToWorld(obj.obstacleLocations);
        end
        
        function [complete, stats] = getCompletionStatus(obj)
            %GETCOMPLETIONSTATUS Check if all mines have been found
            %   [complete, stats] = getCompletionStatus(obj)
            
            stats.totalMines = sum(obj.mineMap(:));
            stats.markedMines = sum(obj.markedMap(:) & obj.mineMap(:));
            stats.falseMarks = sum(obj.markedMap(:) & ~obj.mineMap(:));
            stats.revealedCells = sum(obj.revealedMap(:));
            stats.totalCells = obj.gridRows * obj.gridCols;
            
            complete = (stats.markedMines == stats.totalMines);
        end
        
        function displayStats(obj)
            %DISPLAYSTATS Display current minefield statistics
            [complete, stats] = obj.getCompletionStatus();
            
            fprintf('\n=== Minefield Status ===\n');
            fprintf('Grid Size: %d x %d\n', obj.gridRows, obj.gridCols);
            fprintf('Total Mines: %d\n', stats.totalMines);
            fprintf('Mines Found: %d / %d\n', stats.markedMines, stats.totalMines);
            fprintf('False Marks: %d\n', stats.falseMarks);
            fprintf('Cells Revealed: %d / %d (%.1f%%)\n', ...
                stats.revealedCells, stats.totalCells, ...
                100 * stats.revealedCells / stats.totalCells);
            fprintf('Complete: %s\n', string(complete));
            fprintf('========================\n\n');
        end
    end
end
