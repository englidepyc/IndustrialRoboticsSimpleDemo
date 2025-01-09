function [q, qd, qdd] = trapezoidal_trajectory(q0, qf, t, tf, tc)

% PARAMETER EXPLANATION
    % q0: Vector of initial positions (1 x n joints)
    % qf: Vector of final positions (1 x n joints)
    % t: Time vector (1 x m) 
    % tf: Total time (scalar) 
    % tc: Acceleration time (scalar)


 % EXTRACT PARAMETERS FROM INPUT

    n = length(q0); % Number of joints
    m = length(t); % Number of time steps

    % Compute the constant maximum velocity for each joint (1 x n)
    qdc = (qf - q0) / (tf - tc); 

    
% CHECK FOR TRIANGULAR OR TRAPEZODIAL PROFILE

    % Calculate the minimum required acceleration for trapezoidal trajectory
    min_qddc = 4 * abs(qf - q0) / tf^2;

    % Calculate the max acceleration (absolute)
    qddc = abs(qdc / tc); 

    % If qddc is smaller than the minimum required, adjust or warn
    if (qddc) == (min_qddc)
        disp('Warning: The acceleration is too small for a trapezoidal trajectory.');
        disp('The trajectory may be triangular instead.');
        %qddc = min_qddc;  % Adjust qddc to the minimum value (optional)
    end

   
% INITIALIZE OUTPUT MATRICES
   
    q = zeros(m, n);   % Positions
    qd = zeros(m, n);  % Velocities
    qdd = zeros(m, n); % Accelerations

% COMPUTE Q, QD and QDD FOR EVERY JOINT AND TIME STEP

    % Loop through each joint
    for joint = 1:n
        for i = 1:m %loop through every time step
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



%FURTHER COMMENTS 

% defining tf in the function would not be necessary, we can also extrac
% that from the max value of t(?)

% in the script we worked with qddc (= qdc/tc) but the formulas are the same

%To build a safer code it could also be checked if tc < tf/2
   
    