classdef ROS2TopicManager < handle
    %ROS2TOPICMANAGER Step 2: Manages all ROS2 publishers and subscribers
    %   Central manager for ROS2 communication
    
    properties
        node                % Main ROS2 node
        
        % Publishers
        pubMap              % /map - nav_msgs/OccupancyGrid
        pubRobotPose        % /robot_pose - geometry_msgs/PoseStamped
        pubCmdVel           % /cmd_vel - geometry_msgs/Twist
        pubPath             % /planned_path - nav_msgs/Path
        
        % Subscribers
        subScan             % /scan
        subOdom             % /odom
        subMineAlert        % /mine_alert
        
        % Latest data
        latestScan
        latestOdom
        latestMineAlert
        latestCmdVel
        
        % Callbacks flags
        newScanReceived
        newOdomReceived
        newMineAlertReceived
    end
    
    methods
        function obj = ROS2TopicManager(nodeName)
            %ROS2TOPICMANAGER Create ROS2 topic manager
            
            if nargin < 1
                nodeName = '/minesweeper_main';
            end
            
            fprintf('[ROS2TopicManager] Creating node: %s\n', nodeName);
            obj.node = ros2node(nodeName);
            
            % Initialize flags
            obj.newScanReceived = false;
            obj.newOdomReceived = false;
            obj.newMineAlertReceived = false;
            
            % Initialize latest messages
            obj.latestMineAlert = false;
            obj.latestCmdVel = [0, 0, 0];
            
            % Create publishers
            obj.createPublishers();
            
            % Create subscribers
            obj.createSubscribers();
            
            fprintf('[ROS2TopicManager] Initialized successfully\n');
        end
        
        function createPublishers(obj)
            %CREATEPUBLISHERS Create all publishers
            
            obj.pubMap = ros2publisher(obj.node, '/map', 'nav_msgs/OccupancyGrid');
            obj.pubRobotPose = ros2publisher(obj.node, '/robot_pose', 'geometry_msgs/PoseStamped');
            obj.pubCmdVel = ros2publisher(obj.node, '/cmd_vel', 'geometry_msgs/Twist');
            obj.pubPath = ros2publisher(obj.node, '/planned_path', 'nav_msgs/Path');
            
            fprintf('[ROS2TopicManager] Publishers created:\n');
            fprintf('  /map (nav_msgs/OccupancyGrid)\n');
            fprintf('  /robot_pose (geometry_msgs/PoseStamped)\n');
            fprintf('  /cmd_vel (geometry_msgs/Twist)\n');
            fprintf('  /planned_path (nav_msgs/Path)\n');
        end
        
        function createSubscribers(obj)
            %CREATESUBSCRIBERS Create all subscribers
            
            obj.subScan = ros2subscriber(obj.node, '/scan', 'sensor_msgs/LaserScan', ...
                @obj.scanCallback);
            obj.subOdom = ros2subscriber(obj.node, '/odom', 'nav_msgs/Odometry', ...
                @obj.odomCallback);
            obj.subMineAlert = ros2subscriber(obj.node, '/mine_alert', 'std_msgs/Bool', ...
                @obj.mineAlertCallback);
            
            fprintf('[ROS2TopicManager] Subscribers created:\n');
            fprintf('  /scan (sensor_msgs/LaserScan)\n');
            fprintf('  /odom (nav_msgs/Odometry)\n');
            fprintf('  /mine_alert (std_msgs/Bool)\n');
        end
        
        function scanCallback(obj, msg)
            %SCANCALLBACK Handle incoming laser scan
            obj.latestScan = msg;
            obj.newScanReceived = true;
        end
        
        function odomCallback(obj, msg)
            %ODOMCALLBACK Handle incoming odometry
            obj.latestOdom = msg;
            obj.newOdomReceived = true;
        end
        
        function mineAlertCallback(obj, msg)
            %MINEALERTCALLBACK Handle incoming mine alert
            obj.latestMineAlert = msg.data;
            obj.newMineAlertReceived = true;
        end
        
        function publishMap(obj, world)
            %PUBLISHMAP Publish occupancy grid map
            
            msg = ros2message('nav_msgs/OccupancyGrid');
            
            % Header
            msg.header.stamp = ros2time(obj.node, 'now');
            msg.header.frame_id = 'map';
            
            % Map info
            msg.info.resolution = single(1.0 / world.resolution);
            msg.info.width = uint32(world.map.GridSize(2));
            msg.info.height = uint32(world.map.GridSize(1));
            msg.info.origin.position.x = 0;
            msg.info.origin.position.y = 0;
            msg.info.origin.position.z = 0;
            msg.info.origin.orientation.w = 1;
            
            % Occupancy data
            occMatrix = occupancyMatrix(world.map);
            msg.data = int8(occMatrix(:) * 100);
            
            send(obj.pubMap, msg);
        end
        
        function publishRobotPose(obj, pose)
            %PUBLISHROBOTPOSE Publish robot pose
            %   pose: [x, y, theta]
            
            msg = ros2message('geometry_msgs/PoseStamped');
            
            msg.header.stamp = ros2time(obj.node, 'now');
            msg.header.frame_id = 'map';
            
            msg.pose.position.x = pose(1);
            msg.pose.position.y = pose(2);
            msg.pose.position.z = 0;
            
            % Quaternion from yaw
            msg.pose.orientation.x = 0;
            msg.pose.orientation.y = 0;
            msg.pose.orientation.z = sin(pose(3)/2);
            msg.pose.orientation.w = cos(pose(3)/2);
            
            send(obj.pubRobotPose, msg);
        end
        
        function publishCmdVel(obj, linear, angular)
            %PUBLISHCMDVEL Publish velocity command
            
            msg = ros2message('geometry_msgs/Twist');
            
            msg.linear.x = linear;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = angular;
            
            send(obj.pubCmdVel, msg);
        end
        
        function publishPath(obj, waypoints)
            %PUBLISHPATH Publish planned path
            %   waypoints: [N x 2] array of [x, y] positions
            
            msg = ros2message('nav_msgs/Path');
            
            msg.header.stamp = ros2time(obj.node, 'now');
            msg.header.frame_id = 'map';
            
            % Create pose array
            poses = cell(size(waypoints, 1), 1);
            for i = 1:size(waypoints, 1)
                poseMsg = ros2message('geometry_msgs/PoseStamped');
                poseMsg.header = msg.header;
                poseMsg.pose.position.x = waypoints(i, 1);
                poseMsg.pose.position.y = waypoints(i, 2);
                poseMsg.pose.position.z = 0;
                poseMsg.pose.orientation.w = 1;
                poses{i} = poseMsg;
            end
            msg.poses = poses;
            
            send(obj.pubPath, msg);
        end
        
        function [scan, odom, mineAlert] = getLatestData(obj)
            %GETLATESTDATA Get all latest sensor data
            scan = obj.latestScan;
            odom = obj.latestOdom;
            mineAlert = obj.latestMineAlert;
        end
        
        function pose = extractPoseFromOdom(obj)
            %EXTRACTPOSEFROMODOM Extract [x, y, theta] from odometry
            
            if isempty(obj.latestOdom)
                pose = [0, 0, 0];
                return;
            end
            
            x = obj.latestOdom.pose.pose.position.x;
            y = obj.latestOdom.pose.pose.position.y;
            
            % Extract yaw from quaternion
            qz = obj.latestOdom.pose.pose.orientation.z;
            qw = obj.latestOdom.pose.pose.orientation.w;
            theta = 2 * atan2(qz, qw);
            
            pose = [x, y, theta];
        end
        
        function shutdown(obj)
            %SHUTDOWN Clean shutdown
            fprintf('[ROS2TopicManager] Shutting down...\n');
            clear obj.node;
        end
        
        function delete(obj)
            obj.shutdown();
        end
    end
end
