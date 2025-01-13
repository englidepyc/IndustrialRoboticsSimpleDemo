function [q, qd, qdd] = trapezoidal_trajectory_with_velocity_limit(q0, qf, t, tc, max_velocity)

% TRAPEZOIDAL_TRAJECTORY Generates a trapezoidal trajectory for multiple joints.
% Checks are included to ensure that the maximum velocity does not exceed a specified limit.

% PARAMETER EXPLANATION
    % q0: Vector of initial positions (1 x n joints)
    % qf: Vector of final positions (1 x n joints)
    % t: Time vector (1 x m)  
    % tc: Acceleration time (scalar)
    % max_velocity: Maximum allowable velocity for each joint (scalar or 1 x n vector)

% DEFAULT MAXIMUM VELOCITY
    if nargin < 5
        max_velocity = inf; % No velocity limit if not specified
    end

% EXTRACT PARAMETERS FROM INPUT
    n = length(q0); % Number of joints
    m = length(t);  % Number of time steps
    tf = max(t);    % Final time

% CHECK FOR TRIANGULAR OR TRAPEZOIDAL PROFILE
    if tf < (tc * 2)
        disp('Warning: The acceleration time tc is too high.');
        disp('To reach the maximum velocity, tc has been set to (tf / 2) - 0.5.');
        tc = (tf / 2) - 0.5;
    end

% COMPUTE THE CONSTANT MAXIMUM VELOCITY FOR EACH JOINT (1 x n)
    qdc = (qf - q0) / (tf - tc); 

% CHECK IF MAXIMUM VELOCITY IS EXCEEDED
    for joint = 1:n
        if abs(qdc(joint)) > max_velocity
           disp(['Joint ', num2str(joint), ...
          ': Maximum allowed joint velocity exceeded! ', ...
          'qdc = ', num2str(rad2deg(abs(qdc(joint)))), ' deg/s, ', ...
          'max allowed velocity = ', num2str(rad2deg(max_velocity)), ' deg/s']);
        end
    end

% CALCULATE THE MINIMUM REQUIRED ACCELERATION FOR TRAPEZOIDAL TRAJECTORY
    min_qddc = 4 * abs(qf - q0) / tf^2;

% CALCULATE THE MAXIMUM ACCELERATION
    qddc = qdc / tc; 

% CHECK IF THE TRAJECTORY IS TRIANGULAR
    if any(abs(qddc) == min_qddc)
        disp('Warning: The acceleration is too small for a trapezoidal trajectory.');
        disp('The trajectory may be triangular instead.');
    end

% INITIALIZE OUTPUT MATRICES  
    q = zeros(m, n);   % Positions
    qd = zeros(m, n);  % Velocities
    qdd = zeros(m, n); % Accelerations

% COMPUTE Q, QD AND QDD FOR EVERY JOINT AND TIME STEP
    for joint = 1:n
        for i = 1:m
            if t(i) <= tc
                % Acceleration phase
                q(i, joint) = q0(joint) + qdc(joint) * t(i)^2 / (2 * tc); 
                qd(i, joint) = qdc(joint) * t(i) / tc;
                qdd(i, joint) = qdc(joint) / tc;
            elseif t(i) <= (tf - tc)
                % Constant velocity phase
                q(i, joint) = q0(joint) + qdc(joint) * (t(i) - tc / 2);
                qd(i, joint) = qdc(joint);
                qdd(i, joint) = 0;
            elseif t(i) <= tf
                % Deceleration phase
                q(i, joint) = qf(joint) - qdc(joint) * (tf - t(i))^2 / (2 * tc);
                qd(i, joint) = qdc(joint) * (tf - t(i)) / tc;
                qdd(i, joint) = -qdc(joint) / tc;
            end
        end
    end
end
