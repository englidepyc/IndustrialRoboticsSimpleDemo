function [q, qd, qdd] = trapezoidal_trajectory_with_speed_percentage(q0, qf, t, tc, speed_percentage)
% TRAPEZOIDAL_TRAJECTORY Generates a trapezoidal trajectory for multiple joints.
% This function calculates position (q), velocity (qd), and acceleration (qdd) for each joint over time.
% The position is computed by numerically integrating the velocity to ensure smooth transitions.

% PARAMETER EXPLANATION
    % q0: Vector of initial positions (1 x n joints)
    % qf: Vector of final positions (1 x n joints)
    % t: Time vector (1 x m)  
    % tc: Acceleration time (scalar)
    % speed_percentage: Maximum velocity as a percentage of the theoretical maximum velocity (scalar, 0 to 100)

% INITIALIZATION
    n = length(q0); % Number of joints
    m = length(t);  % Number of time steps
    tf = max(t);    % Final time

% DEFAULT SPEED PERCENTAGE (if not specified)
    if nargin < 5
        speed_percentage = 100; % Default to 100% if not provided
    end

    % Normalize speed percentage to a fraction (0 to 1)
    speed_fraction = min(max(speed_percentage / 100, 0), 1);

% COMPUTE MAXIMUM VELOCITY AND ACCELERATION
    % Maximum theoretical velocity for each joint (1 x n)
    qdc_max = (qf - q0) / (tf - tc); 

    % Scale velocity by speed percentage
    qdc = speed_fraction * qdc_max;

    % Acceleration during the acceleration phase (1 x n)
    qddc = qdc / tc;

% INITIALIZE OUTPUT MATRICES  
    q = zeros(m, n);   % Positions
    qd = zeros(m, n);  % Velocities
    qdd = zeros(m, n); % Accelerations

% COMPUTE Q, QD AND QDD FOR EACH JOINT
    % Loop through each joint
    for joint = 1:n
        % Loop through each time step
        for i = 1:m
            if t(i) <= tc
                % Acceleration phase
                qd(i, joint) = qddc(joint) * t(i); % Velocity
                qdd(i, joint) = qddc(joint);       % Acceleration
            elseif t(i) <= (tf - tc)
                % Constant velocity phase
                qd(i, joint) = qdc(joint); % Velocity remains constant
                qdd(i, joint) = 0;         % No acceleration
            else
                % Deceleration phase
                qd(i, joint) = qddc(joint) * (tf - t(i)); % Velocity decreases
                qdd(i, joint) = -qddc(joint);             % Negative acceleration
            end
        end

        % Compute position by integrating the velocity
        for i = 2:m
            q(i, joint) = q(i-1, joint) + qd(i-1, joint) * (t(i) - t(i-1));
        end
    end
end
