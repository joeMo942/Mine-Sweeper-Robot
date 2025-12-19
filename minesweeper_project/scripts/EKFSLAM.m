classdef EKFSLAM < handle
    %EKFSLAM Step 3: Extended Kalman Filter SLAM
    %   Subscribes to /scan and /odom, publishes robot pose and updated map
    
    properties
        % State vector [x, y, theta]
        state
        
        % Covariance matrix
        covariance
        
        % Process noise
        Q
        
        % Measurement noise
        R
        
        % Map
        landmarks           % [N x 2] detected landmarks
        landmarkCovariances % Cell array of 2x2 covariances
        
        % World reference
        world
        
        % Configuration
        associationThreshold  % Max distance for landmark association
        newLandmarkThreshold  % Min distance to create new landmark
    end
    
    methods
        function obj = EKFSLAM(world)
            %EKFSLAM Create EKF-SLAM instance
            
            obj.world = world;
            
            % Initialize state at start position
            obj.state = [world.startPosition, 0]';  % [x, y, theta]
            
            % Initialize covariance (low initial uncertainty)
            obj.covariance = diag([0.1, 0.1, 0.01]);
            
            % Process noise (motion model uncertainty)
            obj.Q = diag([0.02, 0.02, 0.01]);  % [x, y, theta] noise
            
            % Measurement noise (lidar)
            obj.R = diag([0.1, 0.05]);  % [range, bearing] noise
            
            % Landmarks
            obj.landmarks = [];
            obj.landmarkCovariances = {};
            
            % Thresholds
            obj.associationThreshold = 1.0;  % 1m
            obj.newLandmarkThreshold = 0.5;  % 0.5m
            
            fprintf('[EKFSLAM] Initialized at position (%.2f, %.2f)\n', ...
                    obj.state(1), obj.state(2));
        end
        
        function predict(obj, odomMsg, dt)
            %PREDICT EKF prediction step using odometry
            
            if isempty(odomMsg)
                return;
            end
            
            % Extract velocity from odometry
            vx = odomMsg.twist.twist.linear.x;
            vy = odomMsg.twist.twist.linear.y;
            omega = odomMsg.twist.twist.angular.z;
            
            % Current state
            x = obj.state(1);
            y = obj.state(2);
            theta = obj.state(3);
            
            % Motion model (velocity model)
            v = sqrt(vx^2 + vy^2);
            
            if abs(omega) < 1e-6
                % Straight line motion
                x_new = x + v * cos(theta) * dt;
                y_new = y + v * sin(theta) * dt;
                theta_new = theta;
            else
                % Arc motion
                x_new = x + (-v/omega * sin(theta) + v/omega * sin(theta + omega*dt));
                y_new = y + (v/omega * cos(theta) - v/omega * cos(theta + omega*dt));
                theta_new = theta + omega * dt;
            end
            
            % Normalize angle
            theta_new = wrapToPi(theta_new);
            
            % Update state
            obj.state = [x_new; y_new; theta_new];
            
            % Jacobian of motion model
            if abs(omega) < 1e-6
                G = [1, 0, -v * sin(theta) * dt;
                     0, 1,  v * cos(theta) * dt;
                     0, 0, 1];
            else
                G = [1, 0, -v/omega * cos(theta) + v/omega * cos(theta + omega*dt);
                     0, 1, -v/omega * sin(theta) + v/omega * sin(theta + omega*dt);
                     0, 0, 1];
            end
            
            % Update covariance
            obj.covariance = G * obj.covariance * G' + obj.Q;
        end
        
        function update(obj, scanMsg)
            %UPDATE EKF update step using lidar scan
            
            if isempty(scanMsg)
                return;
            end
            
            % Extract features from scan (simple peak detection)
            features = obj.extractFeatures(scanMsg);
            
            % Update with each feature
            for i = 1:size(features, 1)
                range = features(i, 1);
                bearing = features(i, 2);
                
                % Convert to world coordinates
                x = obj.state(1) + range * cos(obj.state(3) + bearing);
                y = obj.state(2) + range * sin(obj.state(3) + bearing);
                
                % Data association
                [landmarkIdx, minDist] = obj.associateLandmark([x, y]);
                
                if landmarkIdx > 0
                    % Update existing landmark
                    obj.updateWithLandmark(landmarkIdx, range, bearing);
                elseif minDist > obj.newLandmarkThreshold
                    % Add new landmark
                    obj.addLandmark([x, y]);
                end
            end
        end
        
        function features = extractFeatures(obj, scanMsg)
            %EXTRACTFEATURES Extract features from lidar scan
            %   Simple implementation: detect discontinuities
            
            ranges = double(scanMsg.ranges);
            angleMin = double(scanMsg.angle_min);
            angleInc = double(scanMsg.angle_increment);
            
            features = [];
            
            % Find discontinuities (potential corners/objects)
            for i = 2:length(ranges)-1
                if abs(ranges(i) - ranges(i-1)) > 0.5 || ...
                   abs(ranges(i) - ranges(i+1)) > 0.5
                    
                    if ranges(i) < double(scanMsg.range_max) - 0.1
                        bearing = angleMin + (i-1) * angleInc;
                        features(end+1, :) = [ranges(i), bearing];
                    end
                end
            end
            
            % Limit number of features
            if size(features, 1) > 20
                idx = randperm(size(features, 1), 20);
                features = features(idx, :);
            end
        end
        
        function [idx, minDist] = associateLandmark(obj, position)
            %ASSOCIATELANDMARK Find matching landmark
            
            idx = -1;
            minDist = inf;
            
            if isempty(obj.landmarks)
                return;
            end
            
            distances = vecnorm(obj.landmarks - position, 2, 2);
            [minDist, minIdx] = min(distances);
            
            if minDist < obj.associationThreshold
                idx = minIdx;
            end
        end
        
        function addLandmark(obj, position)
            %ADDLANDMARK Add new landmark
            
            obj.landmarks(end+1, :) = position;
            obj.landmarkCovariances{end+1} = eye(2) * 0.5;
        end
        
        function updateWithLandmark(obj, idx, range, bearing)
            %UPDATEWITHLANDMARK Update state using landmark observation
            
            landmark = obj.landmarks(idx, :);
            
            % Expected measurement
            dx = landmark(1) - obj.state(1);
            dy = landmark(2) - obj.state(2);
            expectedRange = sqrt(dx^2 + dy^2);
            expectedBearing = wrapToPi(atan2(dy, dx) - obj.state(3));
            
            % Innovation
            innovation = [range - expectedRange; wrapToPi(bearing - expectedBearing)];
            
            % Measurement Jacobian
            q = dx^2 + dy^2;
            sqrtQ = sqrt(q);
            
            H = [-dx/sqrtQ, -dy/sqrtQ, 0;
                  dy/q,     -dx/q,    -1];
            
            % Kalman gain
            S = H * obj.covariance * H' + obj.R;
            K = obj.covariance * H' / S;
            
            % Update state
            obj.state = obj.state + K * innovation;
            obj.state(3) = wrapToPi(obj.state(3));
            
            % Update covariance
            I = eye(3);
            obj.covariance = (I - K * H) * obj.covariance;
        end
        
        function updateMap(obj, world)
            %UPDATEMAP Update occupancy map with current observation
            
            % Mark robot's current position as explored
            robotPos = obj.state(1:2)';
            
            % Check for mine proximity and mark hazard
            if world.checkMineProximity(robotPos, 0.5)
                [minePos, ~] = world.getNearestMine(robotPos);
                if ~isempty(minePos)
                    % Check if already marked
                    alreadyMarked = false;
                    for i = 1:size(world.hazardZones, 1)
                        if norm(world.hazardZones(i, 1:2) - minePos) < 0.3
                            alreadyMarked = true;
                            break;
                        end
                    end
                    
                    if ~alreadyMarked
                        world.markHazard(minePos(1), minePos(2), 0.5);
                    end
                end
            end
        end
        
        function pose = getPose(obj)
            %GETPOSE Get current pose estimate
            pose = obj.state';
        end
        
        function cov = getCovariance(obj)
            %GETCOVARIANCE Get current covariance
            cov = obj.covariance;
        end
        
        function reset(obj, position, heading)
            %RESET Reset EKF state
            obj.state = [position(:); heading];
            obj.covariance = diag([0.1, 0.1, 0.01]);
            obj.landmarks = [];
            obj.landmarkCovariances = {};
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
