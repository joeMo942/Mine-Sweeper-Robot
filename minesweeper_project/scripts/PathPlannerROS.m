classdef PathPlannerROS < handle
    %PATHPLANNERROS Step 5: Path Planning Node with ROS2
    %   Subscribes to /map and current pose, publishes /cmd_vel
    
    properties
        % Configuration
        inflationRadius     % Obstacle inflation radius
        goalTolerance       % Distance tolerance to goal
        maxLinearVel        % Maximum linear velocity
        maxAngularVel       % Maximum angular velocity
        
        % Path planning
        planner             % Path planner object
        currentPath         % Current planned path [N x 2]
        currentWaypointIdx  % Current waypoint index
        
        % Controller gains
        kpLinear            % Linear velocity gain
        kpAngular           % Angular velocity gain
        
        % State
        isPathValid         % Is current path valid
        needsReplan         % Flag to trigger replanning
    end
    
    methods
        function obj = PathPlannerROS()
            %PATHPLANNERROS Create path planner
            
            obj.inflationRadius = 0.3;  % 30cm inflation
            obj.goalTolerance = 0.3;    % 30cm tolerance
            obj.maxLinearVel = 0.5;     % 0.5 m/s
            obj.maxAngularVel = 1.0;    % 1 rad/s
            
            obj.kpLinear = 1.0;
            obj.kpAngular = 2.0;
            
            obj.currentPath = [];
            obj.currentWaypointIdx = 1;
            obj.isPathValid = false;
            obj.needsReplan = true;
            
            fprintf('[PathPlannerROS] Initialized\n');
        end
        
        function path = planPath(obj, world, startPos, goalPos)
            %PLANPATH Plan path from start to goal avoiding obstacles
            
            fprintf('[PathPlannerROS] Planning path from (%.1f, %.1f) to (%.1f, %.1f)\n', ...
                    startPos(1), startPos(2), goalPos(1), goalPos(2));
            
            % Create inflated map for planning
            mapInflated = copy(world.map);
            inflate(mapInflated, obj.inflationRadius);
            
            % Validate start and goal
            startValid = checkOccupancy(mapInflated, startPos) < 0.5;
            goalValid = checkOccupancy(mapInflated, goalPos) < 0.5;
            
            if ~startValid
                warning('[PathPlannerROS] Start position is in obstacle!');
                path = [];
                return;
            end
            
            if ~goalValid
                warning('[PathPlannerROS] Goal position is in obstacle!');
                path = [];
                return;
            end
            
            try
                % Create binary occupancy map for planner
                binMap = binaryOccupancyMap(mapInflated);
                
                % Create planner (A* or PRM)
                obj.planner = plannerAStarGrid(binMap);
                
                % Plan path
                startGrid = world2grid(binMap, startPos);
                goalGrid = world2grid(binMap, goalPos);
                
                pathGrid = plan(obj.planner, startGrid, goalGrid);
                
                if isempty(pathGrid)
                    warning('[PathPlannerROS] No path found!');
                    path = [];
                    obj.isPathValid = false;
                    return;
                end
                
                % Convert to world coordinates
                path = grid2world(binMap, pathGrid);
                
                % Smooth path (simple moving average)
                if size(path, 1) > 3
                    path = obj.smoothPath(path);
                end
                
                obj.currentPath = path;
                obj.currentWaypointIdx = 1;
                obj.isPathValid = true;
                obj.needsReplan = false;
                
                fprintf('[PathPlannerROS] Path planned with %d waypoints\n', size(path, 1));
                
            catch ME
                warning('[PathPlannerROS] Planning failed: %s', ME.message);
                path = [];
                obj.isPathValid = false;
            end
        end
        
        function path = planCoveragePath(obj, world)
            %PLANCOVERAGEPATH Plan coverage path for exploration
            
            fprintf('[PathPlannerROS] Planning coverage path...\n');
            
            % Simple boustrophedon coverage
            path = [];
            stepSize = 1.0;  % 1m between rows
            
            direction = 1;
            for y = 1:stepSize:world.mapHeight-1
                if direction == 1
                    xRange = 1:stepSize:world.mapWidth-1;
                else
                    xRange = world.mapWidth-1:-stepSize:1;
                end
                
                for x = xRange
                    % Check if position is free
                    if getOccupancy(world.map, [x, y]) < 0.5
                        path(end+1, :) = [x, y];
                    end
                end
                
                direction = -direction;
            end
            
            obj.currentPath = path;
            obj.currentWaypointIdx = 1;
            obj.isPathValid = true;
            obj.needsReplan = false;
            
            fprintf('[PathPlannerROS] Coverage path with %d waypoints\n', size(path, 1));
        end
        
        function replanAroundHazard(obj, world, currentPos, hazardPos)
            %REPLANAROUND HAZARD Replan path to avoid newly marked hazard
            
            fprintf('[PathPlannerROS] Replanning around hazard at (%.1f, %.1f)\n', ...
                    hazardPos(1), hazardPos(2));
            
            if isempty(obj.currentPath) || obj.currentWaypointIdx > size(obj.currentPath, 1)
                return;
            end
            
            % Get remaining goal
            goalPos = obj.currentPath(end, :);
            
            % Replan from current position
            obj.planPath(world, currentPos, goalPos);
        end
        
        function [v, w] = computeVelocity(obj, currentPose)
            %COMPUTEVELOCITY Compute velocity to follow path
            %   currentPose: [x, y, theta]
            %   Returns: v (linear), w (angular)
            
            if ~obj.isPathValid || isempty(obj.currentPath)
                v = 0;
                w = 0;
                return;
            end
            
            if obj.currentWaypointIdx > size(obj.currentPath, 1)
                v = 0;
                w = 0;
                return;
            end
            
            % Get current waypoint
            waypoint = obj.currentPath(obj.currentWaypointIdx, :);
            
            % Compute distance and angle to waypoint
            dx = waypoint(1) - currentPose(1);
            dy = waypoint(2) - currentPose(2);
            distance = sqrt(dx^2 + dy^2);
            
            % Check if waypoint reached
            if distance < obj.goalTolerance
                obj.currentWaypointIdx = obj.currentWaypointIdx + 1;
                
                if obj.currentWaypointIdx > size(obj.currentPath, 1)
                    fprintf('[PathPlannerROS] Path complete!\n');
                    v = 0;
                    w = 0;
                    return;
                end
                
                % Update to next waypoint
                waypoint = obj.currentPath(obj.currentWaypointIdx, :);
                dx = waypoint(1) - currentPose(1);
                dy = waypoint(2) - currentPose(2);
                distance = sqrt(dx^2 + dy^2);
            end
            
            % Compute desired heading
            desiredHeading = atan2(dy, dx);
            headingError = wrapToPi(desiredHeading - currentPose(3));
            
            % Angular velocity
            w = obj.kpAngular * headingError;
            w = max(-obj.maxAngularVel, min(obj.maxAngularVel, w));
            
            % Linear velocity (reduce when turning)
            if abs(headingError) > 0.3
                v = 0.1 * obj.maxLinearVel;
            else
                v = obj.kpLinear * distance;
                v = min(obj.maxLinearVel, v);
            end
        end
        
        function path = smoothPath(obj, path)
            %SMOOTHPATH Apply simple path smoothing
            
            if size(path, 1) < 3
                return;
            end
            
            % Moving average smoothing
            windowSize = 3;
            smoothedPath = path;
            
            for i = 2:size(path, 1)-1
                startIdx = max(1, i - floor(windowSize/2));
                endIdx = min(size(path, 1), i + floor(windowSize/2));
                smoothedPath(i, :) = mean(path(startIdx:endIdx, :), 1);
            end
            
            path = smoothedPath;
        end
        
        function progress = getProgress(obj)
            %GETPROGRESS Get path following progress (0 to 1)
            
            if isempty(obj.currentPath)
                progress = 0;
                return;
            end
            
            progress = (obj.currentWaypointIdx - 1) / size(obj.currentPath, 1);
        end
        
        function remaining = getRemainingWaypoints(obj)
            %GETREMAININGWAYPOINTS Get number of remaining waypoints
            
            if isempty(obj.currentPath)
                remaining = 0;
                return;
            end
            
            remaining = size(obj.currentPath, 1) - obj.currentWaypointIdx + 1;
        end
        
        function invalidatePath(obj)
            %INVALIDATEPATH Mark current path as invalid
            obj.isPathValid = false;
            obj.needsReplan = true;
        end
    end
end

function angle = wrapToPi(angle)
    while angle > pi, angle = angle - 2*pi; end
    while angle < -pi, angle = angle + 2*pi; end
end
