classdef ROS2Interface < handle
    %ROS2INTERFACE ROS2 communication interface for minesweeper robot
    %   Handles ROS2 publishers, subscribers, and message passing
    
    properties
        % ROS2 node
        node
        
        % Publishers
        pubPose
        pubMineDetector
        pubObstacles
        pubMineMarked
        pubSimStatus
        
        % Subscribers
        subCmdVel
        
        % Latest received messages
        lastCmdVel
        
        % Configuration
        isConnected
        useSimulation  % If true, simulates ROS2 without actual connection
        
        % Topic names
        topics
    end
    
    methods
        function obj = ROS2Interface(varargin)
            %ROS2INTERFACE Construct ROS2 interface
            %   obj = ROS2Interface() - auto-detect ROS2 availability
            %   obj = ROS2Interface(useSimulation) - force simulation mode
            
            % Set topic names
            obj.topics.pose = '/robot/pose';
            obj.topics.cmd_vel = '/robot/cmd_vel';
            obj.topics.mine_detector = '/sensor/mine_detector';
            obj.topics.obstacles = '/sensor/obstacles';
            obj.topics.mine_marked = '/robot/mine_marked';
            obj.topics.sim_status = '/simulation/status';
            
            % Initialize state
            obj.isConnected = false;
            obj.lastCmdVel = struct('linear', struct('x', 0, 'y', 0, 'z', 0), ...
                                    'angular', struct('x', 0, 'y', 0, 'z', 0));
            
            % Check if simulation mode is forced
            if nargin >= 1
                obj.useSimulation = varargin{1};
            else
                obj.useSimulation = ~obj.checkROS2Available();
            end
            
            if obj.useSimulation
                fprintf('[ROS2Interface] Running in SIMULATION mode (no ROS2 connection)\n');
            else
                obj.initializeROS2();
            end
        end
        
        function available = checkROS2Available(obj)
            %CHECKROS2AVAILABLE Check if ROS2 toolbox is available
            available = false;
            try
                if exist('ros2node', 'file') == 2
                    available = true;
                end
            catch
                available = false;
            end
        end
        
        function initializeROS2(obj)
            %INITIALIZEROS2 Initialize ROS2 node and topics
            
            try
                fprintf('[ROS2Interface] Initializing ROS2 node...\n');
                
                % Create ROS2 node
                obj.node = ros2node('/minesweeper_matlab');
                
                % Create publishers
                obj.pubPose = ros2publisher(obj.node, obj.topics.pose, ...
                    'geometry_msgs/Pose2D');
                obj.pubMineDetector = ros2publisher(obj.node, obj.topics.mine_detector, ...
                    'std_msgs/Bool');
                obj.pubMineMarked = ros2publisher(obj.node, obj.topics.mine_marked, ...
                    'geometry_msgs/Point');
                obj.pubSimStatus = ros2publisher(obj.node, obj.topics.sim_status, ...
                    'std_msgs/String');
                
                % Create subscribers
                obj.subCmdVel = ros2subscriber(obj.node, obj.topics.cmd_vel, ...
                    'geometry_msgs/Twist', @obj.cmdVelCallback);
                
                obj.isConnected = true;
                fprintf('[ROS2Interface] ROS2 initialized successfully\n');
                
            catch ME
                warning('[ROS2Interface] Failed to initialize ROS2: %s', ME.message);
                warning('[ROS2Interface] Falling back to simulation mode');
                obj.useSimulation = true;
                obj.isConnected = false;
            end
        end
        
        function cmdVelCallback(obj, msg)
            %CMDVELCALLBACK Callback for cmd_vel subscriber
            obj.lastCmdVel = msg;
        end
        
        function publishPose(obj, x, y, theta)
            %PUBLISHPOSE Publish robot pose
            
            if obj.useSimulation
                % Simulation mode - just log
                return;
            end
            
            try
                msg = ros2message('geometry_msgs/Pose2D');
                msg.x = x;
                msg.y = y;
                msg.theta = theta;
                send(obj.pubPose, msg);
            catch ME
                warning('[ROS2Interface] Failed to publish pose: %s', ME.message);
            end
        end
        
        function publishMineDetected(obj, detected)
            %PUBLISHMINEDETECTED Publish mine detection status
            
            if obj.useSimulation
                return;
            end
            
            try
                msg = ros2message('std_msgs/Bool');
                msg.data = detected;
                send(obj.pubMineDetector, msg);
            catch ME
                warning('[ROS2Interface] Failed to publish mine detection: %s', ME.message);
            end
        end
        
        function publishMineMarked(obj, x, y)
            %PUBLISHMINEMARKED Publish marked mine location
            
            if obj.useSimulation
                return;
            end
            
            try
                msg = ros2message('geometry_msgs/Point');
                msg.x = x;
                msg.y = y;
                msg.z = 0;
                send(obj.pubMineMarked, msg);
            catch ME
                warning('[ROS2Interface] Failed to publish mine marked: %s', ME.message);
            end
        end
        
        function publishSimulationStatus(obj, status)
            %PUBLISHSIMULATIONSTATUS Publish simulation status string
            
            if obj.useSimulation
                return;
            end
            
            try
                msg = ros2message('std_msgs/String');
                msg.data = status;
                send(obj.pubSimStatus, msg);
            catch ME
                warning('[ROS2Interface] Failed to publish status: %s', ME.message);
            end
        end
        
        function [v, w] = getCmdVel(obj)
            %GETCMDVEL Get the latest commanded velocity
            %   [v, w] = getCmdVel(obj)
            %   v: linear velocity, w: angular velocity
            
            if obj.useSimulation
                v = 0;
                w = 0;
                return;
            end
            
            v = obj.lastCmdVel.linear.x;
            w = obj.lastCmdVel.angular.z;
        end
        
        function updateFromRobot(obj, robot)
            %UPDATEFROMROBOT Update ROS2 topics from robot state
            %   updateFromRobot(obj, robot)
            
            pose = robot.getPose();
            obj.publishPose(pose(1), pose(2), pose(3));
            obj.publishMineDetected(robot.mineDetected);
        end
        
        function success = testCommunication(obj)
            %TESTCOMMUNICATION Test ROS2 communication
            
            fprintf('[ROS2Interface] Testing communication...\n');
            
            if obj.useSimulation
                fprintf('[ROS2Interface] Running in simulation mode - test skipped\n');
                success = true;
                return;
            end
            
            try
                % Publish test messages
                obj.publishPose(0, 0, 0);
                obj.publishMineDetected(false);
                obj.publishSimulationStatus('TEST');
                
                pause(0.5);
                
                fprintf('[ROS2Interface] Communication test passed\n');
                success = true;
            catch ME
                warning('[ROS2Interface] Communication test failed: %s', ME.message);
                success = false;
            end
        end
        
        function shutdown(obj)
            %SHUTDOWN Clean up ROS2 resources
            
            if obj.isConnected && ~obj.useSimulation
                fprintf('[ROS2Interface] Shutting down ROS2...\n');
                try
                    clear obj.pubPose obj.pubMineDetector obj.pubObstacles;
                    clear obj.pubMineMarked obj.pubSimStatus obj.subCmdVel;
                    clear obj.node;
                catch
                    % Ignore errors during cleanup
                end
            end
            
            obj.isConnected = false;
            fprintf('[ROS2Interface] Shutdown complete\n');
        end
        
        function delete(obj)
            %DELETE Destructor - clean up resources
            obj.shutdown();
        end
    end
end
