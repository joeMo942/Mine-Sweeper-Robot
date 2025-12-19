classdef Visualization < handle
    %VISUALIZATION Real-time visualization for minesweeper simulation
    %   Provides 2D grid visualization with robot, mines, and obstacles
    
    properties
        % Figure and axes handles
        fig
        ax
        
        % Graphics object handles
        hGrid
        hMines
        hObstacles
        hRobot
        hPath
        hSensorRange
        hDetectedMines
        hMarkedMines
        hWaypoints
        hTarget
        
        % Field reference
        field
        
        % Configuration
        figSize
        showPath
        showSensorRange
        showWaypoints
        
        % Colors
        colors
    end
    
    methods
        function obj = Visualization(field, varargin)
            %VISUALIZATION Construct visualization
            %   obj = Visualization(field)
            %   obj = Visualization(field, 'PropertyName', value, ...)
            
            obj.field = field;
            
            % Default configuration
            obj.figSize = [800, 800];
            obj.showPath = true;
            obj.showSensorRange = true;
            obj.showWaypoints = true;
            
            % Process optional arguments
            for i = 1:2:length(varargin)
                switch lower(varargin{i})
                    case 'figsize'
                        obj.figSize = varargin{i+1};
                    case 'showpath'
                        obj.showPath = varargin{i+1};
                    case 'showsensorrange'
                        obj.showSensorRange = varargin{i+1};
                end
            end
            
            % Define colors
            obj.colors.background = [0.2, 0.2, 0.2];
            obj.colors.grid = [0.4, 0.4, 0.4];
            obj.colors.mine = [1, 0, 0];           % Red
            obj.colors.mineHidden = [0.8, 0.2, 0.2]; % Dark red
            obj.colors.obstacle = [0.5, 0.5, 0.5]; % Gray
            obj.colors.robot = [0.2, 0.6, 1];      % Blue
            obj.colors.path = [0.3, 0.8, 0.3];     % Green
            obj.colors.sensorRange = [1, 1, 0.3];  % Yellow
            obj.colors.detected = [1, 0.5, 0];     % Orange
            obj.colors.marked = [0, 1, 0];         % Bright green
            obj.colors.waypoint = [0.7, 0.7, 1];   % Light blue
            
            % Initialize figure
            obj.initializeFigure();
        end
        
        function initializeFigure(obj)
            %INITIALIZEFIGURE Create and set up the figure
            
            % Create figure
            obj.fig = figure('Name', 'Minesweeper Robot Simulation', ...
                            'NumberTitle', 'off', ...
                            'Position', [100, 100, obj.figSize(1), obj.figSize(2)], ...
                            'Color', obj.colors.background, ...
                            'MenuBar', 'figure', ...
                            'ToolBar', 'figure');
            
            % Create axes
            obj.ax = axes('Parent', obj.fig, ...
                         'Position', [0.08, 0.08, 0.86, 0.86], ...
                         'Color', obj.colors.background, ...
                         'XColor', 'w', 'YColor', 'w', ...
                         'GridColor', obj.colors.grid, ...
                         'Box', 'on');
            
            hold(obj.ax, 'on');
            
            % Set axis properties
            worldWidth = obj.field.gridCols * obj.field.cellSize;
            worldHeight = obj.field.gridRows * obj.field.cellSize;
            
            axis(obj.ax, [0, worldWidth, 0, worldHeight]);
            axis(obj.ax, 'equal');
            xlim(obj.ax, [-0.5, worldWidth + 0.5]);
            ylim(obj.ax, [-0.5, worldHeight + 0.5]);
            
            xlabel(obj.ax, 'X (m)', 'Color', 'w', 'FontSize', 12);
            ylabel(obj.ax, 'Y (m)', 'Color', 'w', 'FontSize', 12);
            title(obj.ax, 'Minesweeper Robot Simulation', 'Color', 'w', 'FontSize', 14);
            
            % Draw grid
            obj.drawGrid();
            
            % Initialize empty graphics objects
            obj.hMines = [];
            obj.hObstacles = [];
            obj.hPath = plot(obj.ax, NaN, NaN, '-', 'Color', obj.colors.path, 'LineWidth', 2);
            obj.hSensorRange = [];
            obj.hRobot = [];
            obj.hDetectedMines = [];
            obj.hMarkedMines = [];
            obj.hWaypoints = plot(obj.ax, NaN, NaN, 'o', 'Color', obj.colors.waypoint, 'MarkerSize', 4);
            obj.hTarget = plot(obj.ax, NaN, NaN, 'p', 'Color', 'y', 'MarkerSize', 15, 'MarkerFaceColor', 'y');
            
            % Draw initial field
            obj.drawField();
        end
        
        function drawGrid(obj)
            %DRAWGRID Draw the grid lines
            
            cellSize = obj.field.cellSize;
            rows = obj.field.gridRows;
            cols = obj.field.gridCols;
            
            % Vertical lines
            for c = 0:cols
                x = c * cellSize;
                plot(obj.ax, [x, x], [0, rows * cellSize], '-', ...
                    'Color', obj.colors.grid, 'LineWidth', 0.5);
            end
            
            % Horizontal lines
            for r = 0:rows
                y = r * cellSize;
                plot(obj.ax, [0, cols * cellSize], [y, y], '-', ...
                    'Color', obj.colors.grid, 'LineWidth', 0.5);
            end
        end
        
        function drawField(obj)
            %DRAWFIELD Draw mines and obstacles
            
            cellSize = obj.field.cellSize;
            
            % Clear existing objects
            delete(obj.hObstacles);
            obj.hObstacles = [];
            
            % Draw obstacles
            [obsR, obsC] = find(obj.field.obstacleMap);
            for i = 1:length(obsR)
                x = (obsC(i) - 1) * cellSize;
                y = (obsR(i) - 1) * cellSize;
                
                h = rectangle(obj.ax, 'Position', [x, y, cellSize, cellSize], ...
                             'FaceColor', obj.colors.obstacle, ...
                             'EdgeColor', 'none');
                obj.hObstacles(end+1) = h;
            end
            
            % Mines are hidden until detected - don't draw them initially
        end
        
        function update(obj, robot, planner)
            %UPDATE Update visualization with current state
            %   update(obj, robot)
            %   update(obj, robot, planner)
            
            if ~isvalid(obj.fig)
                return;
            end
            
            % Update robot path
            if obj.showPath && ~isempty(robot.pathHistory) && size(robot.pathHistory, 1) > 1
                set(obj.hPath, 'XData', robot.pathHistory(:,1), ...
                              'YData', robot.pathHistory(:,2));
            end
            
            % Draw robot
            obj.drawRobot(robot);
            
            % Update detected and marked mines
            obj.updateMines(robot);
            
            % Update waypoints if planner provided
            if nargin >= 3 && ~isempty(planner) && obj.showWaypoints
                obj.updateWaypoints(planner);
            end
            
            % Update title with stats
            obj.updateTitle(robot);
            
            drawnow limitrate;
        end
        
        function drawRobot(obj, robot)
            %DRAWROBOT Draw the robot with heading indicator
            
            % Clear previous robot graphics
            delete(obj.hRobot);
            delete(obj.hSensorRange);
            obj.hRobot = [];
            obj.hSensorRange = [];
            
            pos = robot.position;
            heading = robot.heading;
            
            % Robot body (circle)
            theta = linspace(0, 2*pi, 32);
            radius = robot.params.width / 2;
            xBody = pos(1) + radius * cos(theta);
            yBody = pos(2) + radius * sin(theta);
            
            obj.hRobot(1) = fill(obj.ax, xBody, yBody, obj.colors.robot, ...
                                'EdgeColor', 'w', 'LineWidth', 2);
            
            % Heading indicator (arrow)
            arrowLen = radius * 1.5;
            xHead = [pos(1), pos(1) + arrowLen * cos(heading)];
            yHead = [pos(2), pos(2) + arrowLen * sin(heading)];
            
            obj.hRobot(2) = plot(obj.ax, xHead, yHead, 'w-', 'LineWidth', 3);
            
            % Sensor range circle (if enabled)
            if obj.showSensorRange
                sensorRange = robot.params.mine_detector_range;
                xSensor = pos(1) + sensorRange * cos(theta);
                ySensor = pos(2) + sensorRange * sin(theta);
                
                obj.hSensorRange = plot(obj.ax, xSensor, ySensor, '--', ...
                                       'Color', obj.colors.sensorRange, ...
                                       'LineWidth', 1);
            end
            
            % Current target
            if ~robot.targetReached
                set(obj.hTarget, 'XData', robot.targetPosition(1), ...
                                'YData', robot.targetPosition(2));
            else
                set(obj.hTarget, 'XData', NaN, 'YData', NaN);
            end
        end
        
        function updateMines(obj, robot)
            %UPDATEMINES Update mine visualization
            
            cellSize = obj.field.cellSize;
            
            % Clear previous mine graphics
            delete(obj.hDetectedMines);
            delete(obj.hMarkedMines);
            obj.hDetectedMines = [];
            obj.hMarkedMines = [];
            
            % Draw detected mines (orange circles)
            if ~isempty(robot.detectedMines)
                for i = 1:size(robot.detectedMines, 1)
                    r = robot.detectedMines(i, 1);
                    c = robot.detectedMines(i, 2);
                    x = (c - 0.5) * cellSize;
                    y = (r - 0.5) * cellSize;
                    
                    % Check if also marked
                    isMarked = ~isempty(robot.markedMines) && ...
                              any(all(robot.markedMines == [r, c], 2));
                    
                    if ~isMarked
                        h = plot(obj.ax, x, y, 'o', 'MarkerSize', 20, ...
                                'Color', obj.colors.detected, ...
                                'LineWidth', 3);
                        obj.hDetectedMines(end+1) = h;
                    end
                end
            end
            
            % Draw marked mines (green X with circle)
            if ~isempty(robot.markedMines)
                for i = 1:size(robot.markedMines, 1)
                    r = robot.markedMines(i, 1);
                    c = robot.markedMines(i, 2);
                    x = (c - 0.5) * cellSize;
                    y = (r - 0.5) * cellSize;
                    
                    % Green circle
                    h1 = plot(obj.ax, x, y, 'o', 'MarkerSize', 25, ...
                             'Color', obj.colors.marked, ...
                             'LineWidth', 3);
                    
                    % Green X
                    h2 = plot(obj.ax, x, y, 'x', 'MarkerSize', 20, ...
                             'Color', obj.colors.marked, ...
                             'LineWidth', 3);
                    
                    obj.hMarkedMines = [obj.hMarkedMines, h1, h2];
                end
            end
        end
        
        function updateWaypoints(obj, planner)
            %UPDATEWAYPOINTS Update waypoint visualization
            
            if ~isempty(planner.waypoints) && planner.currentIndex <= size(planner.waypoints, 1)
                % Show remaining waypoints only
                remaining = planner.waypoints(planner.currentIndex:end, :);
                set(obj.hWaypoints, 'XData', remaining(:,1), ...
                                   'YData', remaining(:,2));
            else
                set(obj.hWaypoints, 'XData', NaN, 'YData', NaN);
            end
        end
        
        function updateTitle(obj, robot)
            %UPDATETITLE Update figure title with status
            
            info = robot.getInfo();
            [complete, stats] = obj.field.getCompletionStatus();
            
            statusStr = sprintf('State: %s | Mines: %d/%d | Position: (%.1f, %.1f)', ...
                               info.state, stats.markedMines, stats.totalMines, ...
                               info.position(1), info.position(2));
            
            if complete
                statusStr = ['[COMPLETE] ', statusStr];
            end
            
            title(obj.ax, statusStr, 'Color', 'w', 'FontSize', 12);
        end
        
        function showAllMines(obj)
            %SHOWALLMINES Reveal all mines (for debugging/end of game)
            
            cellSize = obj.field.cellSize;
            
            [mineR, mineC] = find(obj.field.mineMap);
            for i = 1:length(mineR)
                x = (mineC(i) - 0.5) * cellSize;
                y = (mineR(i) - 0.5) * cellSize;
                
                plot(obj.ax, x, y, '*', 'MarkerSize', 15, ...
                    'Color', obj.colors.mine, 'LineWidth', 2);
            end
            
            drawnow;
        end
        
        function saveFrame(obj, filename)
            %SAVEFRAME Save current frame as image
            if isvalid(obj.fig)
                saveas(obj.fig, filename);
            end
        end
        
        function close(obj)
            %CLOSE Close the visualization figure
            if isvalid(obj.fig)
                close(obj.fig);
            end
        end
        
        function delete(obj)
            %DELETE Destructor
            obj.close();
        end
    end
end
