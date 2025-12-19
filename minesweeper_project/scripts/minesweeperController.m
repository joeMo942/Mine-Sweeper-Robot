function [cmd_vel] = minesweeperController(pose, mine_detected, target)
    %MINESWEEPERCONTROLLER Controller function for Simulink
    %   This function computes velocity commands based on robot pose and target
    %
    %   Inputs:
    %       pose - [x, y, theta] current robot pose
    %       mine_detected - boolean indicating mine detection
    %       target - [x, y] target position
    %
    %   Outputs:
    %       cmd_vel - [v, w] linear and angular velocity commands
    
    %#codegen
    
    % Controller parameters
    persistent kp_linear kp_angular max_v max_w goal_tol angle_tol
    
    if isempty(kp_linear)
        kp_linear = 1.0;
        kp_angular = 2.0;
        max_v = 0.5;
        max_w = 1.0;
        goal_tol = 0.1;
        angle_tol = 0.1;
    end
    
    % Extract pose components
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    
    % Extract target
    target_x = target(1);
    target_y = target(2);
    
    % If mine detected, stop
    if mine_detected > 0.5
        cmd_vel = [0, 0];
        return;
    end
    
    % Compute error to target
    dx = target_x - x;
    dy = target_y - y;
    distance = sqrt(dx^2 + dy^2);
    
    % Check if target reached
    if distance < goal_tol
        cmd_vel = [0, 0];
        return;
    end
    
    % Compute desired heading
    desired_heading = atan2(dy, dx);
    
    % Compute heading error (wrapped to [-pi, pi])
    heading_error = desired_heading - theta;
    while heading_error > pi
        heading_error = heading_error - 2*pi;
    end
    while heading_error < -pi
        heading_error = heading_error + 2*pi;
    end
    
    % Compute angular velocity
    w = kp_angular * heading_error;
    w = max(-max_w, min(max_w, w));
    
    % Compute linear velocity (reduce when turning)
    if abs(heading_error) > angle_tol
        v = 0.1 * max_v;
    else
        v = kp_linear * distance;
        v = max(0, min(max_v, v));
    end
    
    % Output velocity command
    cmd_vel = [v, w];
end
