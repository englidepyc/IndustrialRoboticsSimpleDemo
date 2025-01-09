function [q, qd, qdd] = trapezoidal_trajectory(q0, qf, t, tf, tc)
    % q0: Vector of initial positions (1 x n joints)
    % qf: Vector of final positions (1 x n joints)
    % t: Time vector (1 x m) 
    % tf: Total time (scalar) - not necessary, we can also extrac that from
    % the max value of t?
    % tc: Acceleration time (scalar)

    %It could also be added to check that tc < tf/2
    
    n = length(q0); % Number of joints
    m = length(t); % Number of time steps

    % Compute the constant maximum velocity for each joint
    qdc = (qf - q0) / (tf - tc) % Maximum velocity (1 x n)

    % Calculate the minimum required acceleration for trapezoidal trajectory
    min_qddc = 4 * abs(qf - q0) / tf^2

    % Check if qddc is large enough, otherwise adjust or warn
    qddc = abs(qdc / tc) % Max acceleration

    % If qddc is smaller than the minimum required, adjust or warn
    if (qddc) == (min_qddc)
        disp('Warning: The acceleration is too small for a trapezoidal trajectory.');
        disp('The trajectory may be triangular instead.');
        % You can either adjust qddc to the minimum value or leave it as is
        %qddc = min_qddc;  % Adjust qddc to the minimum value (optional)
    end

   

    % Initialize output matrices
    q = zeros(m, n);   % Positions
    qd = zeros(m, n);  % Velocities
    qdd = zeros(m, n); % Accelerations

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

%in the script we worked with qddc (= qdc/tc) but the formulas are the same
   
    