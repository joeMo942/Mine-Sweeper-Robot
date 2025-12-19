classdef MinesweeperRobot < handle
    %MINESWEEPERROBOT Robot class for minesweeper simulation
    %   Handles robot state, movement, and sensor simulation
    
    properties
        % Position and orientation
        position        % [x, y] in meters
        heading         % Heading angle in radians
        velocity        % Current linear velocity
        angularVelocity % Current angular velocity
        
        % Robot parameters
        params          % Robot parameters structure
        
        % State machine
        state           % Current state: 'IDLE', 'MOVING', 'SCANNING', 'MARKING'
        
        % Sensor data
        mineDetected    % Boolean - mine detected at current position
        obstacleDistances % Array of obstacle distances from sensors
        
        % History and tracking
        pathHistory     % [N x 2] array of position history
        detectedMines   % [M x 2] array of detected mine positions
        markedMines     % [K x 2] array of marked mine positions
        
        % Target for movement
        targetPosition  % Current movement target [x, y]
        targetReached   % Boolean - target reached
    end
    
    properties (Constant)
        % State constants
        STATE_IDLE = 'IDLE'
        STATE_MOVING = 'MOVING'
        STATE_SCANNING = 'SCANNING'
        STATE_MARKING = 'MARKING'
        STATE_AVOIDING = 'AVOIDING'
    end
    
    methods
        function obj = MinesweeperRobot(startPosition, startHeading, params)
            %MINESWEEPERROBOT Construct robot instance
            %   obj = MinesweeperRobot(startPosition, startHeading)
            %   obj = MinesweeperRobot(startPosition, startHeading, params)
            
            obj.position = startPosition(:)';
            obj.heading = startHeading;
            obj.velocity = 0;
            obj.angularVelocity = 0;
            
            % Load default parameters if not provided
            if nargin < 3
                obj.params = obj.getDefaultParams();
            else
                obj.params = params;
            end
            
            obj.state = obj.STATE_IDLE;
            obj.mineDetected = false;
            obj.obstacleDistances = inf(1, obj.params.num_obstacle_rays);
            
            obj.pathHistory = startPosition(:)';
            obj.detectedMines = [];
            obj.markedMines = [];
            
            obj.targetPosition = startPosition(:)';
            obj.targetReached = true;
        end
        
        function params = getDefaultParams(~)
            %GETDEFAULTPARAMS Get default robot parameters
            params.max_velocity = 0.5;
            params.max_angular_velocity = 1.0;
            params.width = 0.3;
            params.length = 0.4;
            params.mine_detector_range = 0.5;
            params.mine_detector_accuracy = 0.95;
            params.obstacle_sensor_range = 1.0;
            params.obstacle_sensor_fov = pi/2;
            params.num_obstacle_rays = 8;
            params.kp_linear = 1.0;
            params.kp_angular = 2.0;
            params.goal_tolerance = 0.1;
            params.angle_tolerance = 0.1;
        end
        
        function setTarget(obj, target)
            %SETTARGET Set movement target position
            obj.targetPosition = target(:)';
            obj.targetReached = false;
            obj.state = obj.STATE_MOVING;
        end
        
        function [v, w] = computeControl(obj)
            %COMPUTECONTROL Compute velocity commands to reach target
            %   [v, w] = computeControl(obj)
            %   v: linear velocity, w: angular velocity
            
            % Vector to target
            toTarget = obj.targetPosition - obj.position;
            distance = norm(toTarget);
            
            % Check if target reached
            if distance < obj.params.goal_tolerance
                v = 0;
                w = 0;
                obj.targetReached = true;
                return;
            end
            
            % Desired heading to target
            desiredHeading = atan2(toTarget(2), toTarget(1));
            
            % Heading error (wrapped to [-pi, pi])
            headingError = wrapToPi(desiredHeading - obj.heading);
            
            % Angular velocity control
            w = obj.params.kp_angular * headingError;
            w = max(-obj.params.max_angular_velocity, ...
                    min(obj.params.max_angular_velocity, w));
            
            % Linear velocity control (reduce speed when turning)
            if abs(headingError) > obj.params.angle_tolerance
                v = 0.1 * obj.params.max_velocity;
            else
                v = obj.params.kp_linear * distance;
                v = max(0, min(obj.params.max_velocity, v));
            end
        end
        
        function update(obj, dt, field)
            %UPDATE Update robot state for one time step
            %   update(obj, dt) - update with time step dt
            %   update(obj, dt, field) - update with minefield reference
            
            % Compute control if moving
            if strcmp(obj.state, obj.STATE_MOVING)
                [v, w] = obj.computeControl();
                obj.velocity = v;
                obj.angularVelocity = w;
                
                if obj.targetReached
                    obj.state = obj.STATE_SCANNING;
                    obj.velocity = 0;
                    obj.angularVelocity = 0;
                end
            end
            
            % Update kinematics (differential drive model)
            obj.heading = obj.heading + obj.angularVelocity * dt;
            obj.heading = wrapToPi(obj.heading);
            
            dx = obj.velocity * cos(obj.heading) * dt;
            dy = obj.velocity * sin(obj.heading) * dt;
            obj.position = obj.position + [dx, dy];
            
            % Record path history
            obj.pathHistory(end+1, :) = obj.position;
            
            % Update sensors if field is provided
            if nargin >= 3 && ~isempty(field)
                obj.updateSensors(field);
            end
        end
        
        function updateSensors(obj, field)
            %UPDATESENSORS Update sensor readings based on field
            
            % Get current grid position
            gridPos = field.worldToGrid(obj.position);
            
            % Mine detector - check current cell and neighbors
            obj.mineDetected = false;
            for dr = -1:1
                for dc = -1:1
                    checkRow = gridPos(1) + dr;
                    checkCol = gridPos(2) + dc;
                    if checkRow >= 1 && checkRow <= field.gridRows && ...
                       checkCol >= 1 && checkCol <= field.gridCols
                        
                        cellCenter = field.gridToWorld([checkRow, checkCol]);
                        dist = norm(cellCenter - obj.position);
                        
                        if dist <= obj.params.mine_detector_range
                            if field.mineMap(checkRow, checkCol)
                                % Apply detection accuracy
                                if rand() < obj.params.mine_detector_accuracy
                                    obj.mineDetected = true;
                                    
                                    % Add to detected mines if new
                                    if isempty(obj.detectedMines) || ...
                                       ~any(all(obj.detectedMines == [checkRow, checkCol], 2))
                                        obj.detectedMines(end+1, :) = [checkRow, checkCol];
                                    end
                                end
                            end
                        end
                    end
                end
            end
            
            % Obstacle sensors (simulated ray casting)
            angles = linspace(-obj.params.obstacle_sensor_fov/2, ...
                              obj.params.obstacle_sensor_fov/2, ...
                              obj.params.num_obstacle_rays);
            
            obj.obstacleDistances = ones(1, obj.params.num_obstacle_rays) * ...
                                   obj.params.obstacle_sensor_range;
            
            for i = 1:length(angles)
                rayAngle = obj.heading + angles(i);
                
                % Check obstacles along ray
                for dist = 0.1:0.1:obj.params.obstacle_sensor_range
                    checkPos = obj.position + dist * [cos(rayAngle), sin(rayAngle)];
                    checkGrid = field.worldToGrid(checkPos);
                    
                    % Check bounds
                    if checkGrid(1) < 1 || checkGrid(1) > field.gridRows || ...
                       checkGrid(2) < 1 || checkGrid(2) > field.gridCols
                        obj.obstacleDistances(i) = dist;
                        break;
                    end
                    
                    % Check for obstacle
                    if field.obstacleMap(checkGrid(1), checkGrid(2))
                        obj.obstacleDistances(i) = dist;
                        break;
                    end
                end
            end
        end
        
        function success = markCurrentMine(obj, field)
            %MARKCURRENTMINE Mark mine at current position
            
            success = false;
            if obj.mineDetected && ~isempty(obj.detectedMines)
                % Mark the most recently detected mine
                minePos = obj.detectedMines(end, :);
                success = field.markMine(minePos(1), minePos(2));
                
                if success
                    obj.markedMines(end+1, :) = minePos;
                    fprintf('Mine marked at grid position [%d, %d]\n', minePos(1), minePos(2));
                end
            end
        end
        
        function setState(obj, newState)
            %SETSTATE Change robot state
            obj.state = newState;
        end
        
        function pose = getPose(obj)
            %GETPOSE Get current pose as [x, y, theta]
            pose = [obj.position, obj.heading];
        end
        
        function vel = getVelocity(obj)
            %GETVELOCITY Get current velocity as [v, w]
            vel = [obj.velocity, obj.angularVelocity];
        end
        
        function info = getInfo(obj)
            %GETINFO Get robot information structure
            info.position = obj.position;
            info.heading = obj.heading;
            info.velocity = obj.velocity;
            info.angularVelocity = obj.angularVelocity;
            info.state = obj.state;
            info.mineDetected = obj.mineDetected;
            info.obstacleDistances = obj.obstacleDistances;
            info.numDetectedMines = size(obj.detectedMines, 1);
            info.numMarkedMines = size(obj.markedMines, 1);
        end
    end
end

function angle = wrapToPi(angle)
    %WRAPTOPI Wrap angle to [-pi, pi]
    while angle > pi
        angle = angle - 2*pi;
    end
    while angle < -pi
        angle = angle + 2*pi;
    end
end
