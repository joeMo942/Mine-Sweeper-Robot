classdef SensorSimulator < handle
    %SENSORSIMULATOR Step 2: Simulates Lidar, Odometry, and Mine Detector
    %   Publishes sensor data to ROS2 topics
    
    properties
        % ROS2 node and publishers
        node
        pubScan             % /scan - sensor_msgs/LaserScan
        pubOdom             % /odom - nav_msgs/Odometry
        pubMineAlert        % /mine_alert - std_msgs/Bool
        
        % Sensor parameters
        lidarRange          % Maximum lidar range (m)
        lidarAngleMin       % Minimum angle (rad)
        lidarAngleMax       % Maximum angle (rad)
        lidarNumRays        % Number of lidar rays
        
        mineDetectorRange   % Mine detector range (m)
        
        % Noise parameters
        lidarNoise          % Lidar measurement noise std
        odomNoiseLinear     % Odometry linear noise
        odomNoiseAngular    % Odometry angular noise
        
        % State
        lastOdomTime
        isInitialized
    end
    
    methods
        function obj = SensorSimulator(node)
            %SENSORSIMULATOR Create sensor simulator
            %   obj = SensorSimulator(ros2node)
            
            obj.node = node;
            
            % Lidar parameters
            obj.lidarRange = 10.0;       % 10m range
            obj.lidarAngleMin = -pi;     % -180 degrees
            obj.lidarAngleMax = pi;      % +180 degrees
            obj.lidarNumRays = 360;      % 1 degree resolution
            obj.lidarNoise = 0.02;       % 2cm noise std
            
            % Mine detector
            obj.mineDetectorRange = 0.5; % 0.5m detection range
            
            % Odometry noise
            obj.odomNoiseLinear = 0.01;  % 1cm std
            obj.odomNoiseAngular = 0.01; % ~0.5 deg std
            
            % Create publishers
            obj.pubScan = ros2publisher(obj.node, '/scan', 'sensor_msgs/LaserScan');
            obj.pubOdom = ros2publisher(obj.node, '/odom', 'nav_msgs/Odometry');
            obj.pubMineAlert = ros2publisher(obj.node, '/mine_alert', 'std_msgs/Bool');
            
            obj.lastOdomTime = tic;
            obj.isInitialized = true;
            
            fprintf('[SensorSimulator] Initialized with publishers:\n');
            fprintf('  /scan (sensor_msgs/LaserScan)\n');
            fprintf('  /odom (nav_msgs/Odometry)\n');
            fprintf('  /mine_alert (std_msgs/Bool)\n');
        end
        
        function publishLidarScan(obj, robotPose, world)
            %PUBLISHLIDARSCAN Simulate and publish lidar scan
            %   robotPose: [x, y, theta]
            %   world: OccupancyGridWorld object
            
            % Create LaserScan message
            msg = ros2message('sensor_msgs/LaserScan');
            
            % Header
            msg.header.stamp = ros2time(obj.node, 'now');
            msg.header.frame_id = 'base_scan';
            
            % Scan parameters
            msg.angle_min = single(obj.lidarAngleMin);
            msg.angle_max = single(obj.lidarAngleMax);
            msg.angle_increment = single((obj.lidarAngleMax - obj.lidarAngleMin) / obj.lidarNumRays);
            msg.time_increment = single(0);
            msg.scan_time = single(0.1);
            msg.range_min = single(0.1);
            msg.range_max = single(obj.lidarRange);
            
            % Simulate ranges using ray casting
            angles = linspace(obj.lidarAngleMin, obj.lidarAngleMax, obj.lidarNumRays);
            ranges = zeros(1, obj.lidarNumRays, 'single');
            
            for i = 1:obj.lidarNumRays
                rayAngle = robotPose(3) + angles(i);
                ranges(i) = obj.castRay(robotPose(1:2), rayAngle, world);
            end
            
            % Add noise
            ranges = ranges + randn(size(ranges)) * obj.lidarNoise;
            ranges = max(0.1, min(obj.lidarRange, ranges));
            
            msg.ranges = ranges';
            msg.intensities = ones(obj.lidarNumRays, 1, 'single') * 100;
            
            % Publish
            send(obj.pubScan, msg);
        end
        
        function range = castRay(obj, origin, angle, world)
            %CASTRAY Cast a single ray and return distance to obstacle
            
            stepSize = 0.05;  % 5cm steps
            range = obj.lidarRange;
            
            for d = stepSize:stepSize:obj.lidarRange
                x = origin(1) + d * cos(angle);
                y = origin(2) + d * sin(angle);
                
                % Check bounds
                if x < 0 || x > world.mapWidth || y < 0 || y > world.mapHeight
                    range = d;
                    return;
                end
                
                % Check occupancy
                try
                    occ = getOccupancy(world.map, [x, y]);
                    if occ > 0.5
                        range = d;
                        return;
                    end
                catch
                    range = d;
                    return;
                end
            end
        end
        
        function publishOdometry(obj, robotPose, velocity)
            %PUBLISHODOMETRY Publish odometry message
            %   robotPose: [x, y, theta]
            %   velocity: [vx, vy, omega]
            
            msg = ros2message('nav_msgs/Odometry');
            
            % Header
            msg.header.stamp = ros2time(obj.node, 'now');
            msg.header.frame_id = 'odom';
            msg.child_frame_id = 'base_link';
            
            % Position with noise
            msg.pose.pose.position.x = robotPose(1) + randn() * obj.odomNoiseLinear;
            msg.pose.pose.position.y = robotPose(2) + randn() * obj.odomNoiseLinear;
            msg.pose.pose.position.z = 0;
            
            % Orientation as quaternion
            theta = robotPose(3) + randn() * obj.odomNoiseAngular;
            msg.pose.pose.orientation.x = 0;
            msg.pose.pose.orientation.y = 0;
            msg.pose.pose.orientation.z = sin(theta/2);
            msg.pose.pose.orientation.w = cos(theta/2);
            
            % Velocity
            if nargin >= 3
                msg.twist.twist.linear.x = velocity(1);
                msg.twist.twist.linear.y = velocity(2);
                msg.twist.twist.angular.z = velocity(3);
            end
            
            % Covariance (simplified)
            msg.pose.covariance = zeros(36, 1);
            msg.pose.covariance(1) = obj.odomNoiseLinear^2;
            msg.pose.covariance(8) = obj.odomNoiseLinear^2;
            msg.pose.covariance(36) = obj.odomNoiseAngular^2;
            
            % Publish
            send(obj.pubOdom, msg);
        end
        
        function detected = publishMineAlert(obj, robotPose, world)
            %PUBLISHMINEALERT Check for mine and publish alert
            %   Returns true if mine detected
            
            detected = world.checkMineProximity(robotPose(1:2), obj.mineDetectorRange);
            
            msg = ros2message('std_msgs/Bool');
            msg.data = detected;
            send(obj.pubMineAlert, msg);
        end
        
        function updateAll(obj, robotPose, velocity, world)
            %UPDATEALL Update all sensors
            
            obj.publishLidarScan(robotPose, world);
            obj.publishOdometry(robotPose, velocity);
            obj.publishMineAlert(robotPose, world);
        end
        
        function delete(obj)
            %DELETE Cleanup
            fprintf('[SensorSimulator] Shutting down...\n');
        end
    end
end
