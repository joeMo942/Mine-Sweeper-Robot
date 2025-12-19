classdef StateMachine < handle
    %STATEMACHINE Step 4: FSM for Minesweeper Robot
    %   Implements: Explore, DetectMine, MarkHazard, AvoidZone, ReplanPath
    
    properties
        % Current state
        state
        previousState
        
        % State constants
        EXPLORE
        DETECT_MINE
        MARK_HAZARD
        AVOID_ZONE
        REPLAN_PATH
        MISSION_COMPLETE
        
        % Transition conditions
        mineDetected
        mineConfirmed
        hazardMarked
        pathReplanned
        missionComplete
        
        % Timers
        stateEntryTime
        detectTimeout      % Time to confirm mine detection
        markTimeout        % Time to mark hazard
        
        % Data
        currentMinePosition
        
        % Callbacks (function handles)
        onStateChange
    end
    
    methods
        function obj = StateMachine()
            %STATEMACHINE Create state machine
            
            % Define states
            obj.EXPLORE = 'EXPLORE';
            obj.DETECT_MINE = 'DETECT_MINE';
            obj.MARK_HAZARD = 'MARK_HAZARD';
            obj.AVOID_ZONE = 'AVOID_ZONE';
            obj.REPLAN_PATH = 'REPLAN_PATH';
            obj.MISSION_COMPLETE = 'MISSION_COMPLETE';
            
            % Initialize state
            obj.state = obj.EXPLORE;
            obj.previousState = '';
            
            % Initialize conditions
            obj.mineDetected = false;
            obj.mineConfirmed = false;
            obj.hazardMarked = false;
            obj.pathReplanned = false;
            obj.missionComplete = false;
            
            % Timeouts
            obj.detectTimeout = 1.0;   % 1 second to confirm
            obj.markTimeout = 0.5;     % 0.5 seconds to mark
            
            % Timer
            obj.stateEntryTime = tic;
            
            % Data
            obj.currentMinePosition = [];
            
            % Callback
            obj.onStateChange = [];
            
            fprintf('[StateMachine] Initialized in state: %s\n', obj.state);
        end
        
        function [newState, action] = update(obj, mineAlert, robotPose, world, planner)
            %UPDATE Update state machine
            %   Returns new state and action to take
            
            action = struct('stop', false, 'replan', false, 'markHazard', false, ...
                           'hazardPosition', [], 'continue', true);
            
            switch obj.state
                case obj.EXPLORE
                    % Exploring - following path
                    if mineAlert
                        obj.transitionTo(obj.DETECT_MINE);
                        obj.mineDetected = true;
                        action.stop = true;
                        
                        % Store potential mine position
                        obj.currentMinePosition = robotPose(1:2);
                    end
                    
                    % Check if mission complete
                    if planner.currentWaypointIdx > size(planner.currentPath, 1)
                        stats = world.getStats();
                        if stats.minesRemaining == 0
                            obj.transitionTo(obj.MISSION_COMPLETE);
                        end
                    end
                    
                case obj.DETECT_MINE
                    % Confirming mine detection
                    action.stop = true;
                    
                    elapsed = toc(obj.stateEntryTime);
                    
                    if elapsed > obj.detectTimeout
                        if mineAlert  % Still detecting
                            obj.mineConfirmed = true;
                            obj.transitionTo(obj.MARK_HAZARD);
                            
                            % Get actual mine position
                            [minePos, ~] = world.getNearestMine(robotPose(1:2));
                            if ~isempty(minePos)
                                obj.currentMinePosition = minePos;
                            end
                        else
                            % False alarm
                            obj.transitionTo(obj.EXPLORE);
                            obj.mineDetected = false;
                        end
                    end
                    
                case obj.MARK_HAZARD
                    % Marking the hazard on map
                    action.stop = true;
                    action.markHazard = true;
                    action.hazardPosition = obj.currentMinePosition;
                    
                    elapsed = toc(obj.stateEntryTime);
                    
                    if elapsed > obj.markTimeout
                        obj.hazardMarked = true;
                        obj.transitionTo(obj.AVOID_ZONE);
                    end
                    
                case obj.AVOID_ZONE
                    % Moving away from hazard zone
                    action.stop = false;
                    
                    % Check if we're far enough from hazard
                    distToHazard = norm(robotPose(1:2) - obj.currentMinePosition);
                    
                    if distToHazard > 1.0  % 1m away
                        obj.transitionTo(obj.REPLAN_PATH);
                        action.replan = true;
                    else
                        % Move backwards
                        action.moveBack = true;
                    end
                    
                case obj.REPLAN_PATH
                    % Replanning path to avoid marked hazards
                    action.replan = true;
                    
                    % Once replanning is done (handled externally)
                    if planner.isPathValid && ~planner.needsReplan
                        obj.pathReplanned = true;
                        obj.transitionTo(obj.EXPLORE);
                        
                        % Reset flags
                        obj.mineDetected = false;
                        obj.mineConfirmed = false;
                        obj.hazardMarked = false;
                        obj.pathReplanned = false;
                    end
                    
                case obj.MISSION_COMPLETE
                    % Mission complete
                    action.stop = true;
                    action.continue = false;
                    fprintf('[StateMachine] Mission Complete!\n');
            end
            
            newState = obj.state;
        end
        
        function transitionTo(obj, newState)
            %TRANSITIONTO Transition to new state
            
            if ~strcmp(obj.state, newState)
                fprintf('[StateMachine] %s -> %s\n', obj.state, newState);
                obj.previousState = obj.state;
                obj.state = newState;
                obj.stateEntryTime = tic;
                
                if ~isempty(obj.onStateChange)
                    obj.onStateChange(obj.previousState, obj.state);
                end
            end
        end
        
        function stateStr = getState(obj)
            %GETSTATE Get current state as string
            stateStr = obj.state;
        end
        
        function reset(obj)
            %RESET Reset state machine
            obj.state = obj.EXPLORE;
            obj.previousState = '';
            obj.mineDetected = false;
            obj.mineConfirmed = false;
            obj.hazardMarked = false;
            obj.pathReplanned = false;
            obj.missionComplete = false;
            obj.stateEntryTime = tic;
            obj.currentMinePosition = [];
            fprintf('[StateMachine] Reset to EXPLORE\n');
        end
    end
end
