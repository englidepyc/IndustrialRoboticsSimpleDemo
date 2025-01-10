function [q, qd, qdd] = trapezoidal_trajectory(q0, qf, t, tf, tc)
    % q0: Vector of initial positions (1 x n joints)
    % qf: Vector of final positions (1 x n joints)
    % t: Time vector (1 x m) 
    % tf: Total time (scalar) - not necessary, we can also extrac that from
    % the max value of t?
    % tc: Acceleration time (scalar)

    
    n = length(q0); % Number of joints
    m = length(t); % Number of time steps

    % Compute the constant velocity for each joint
    spp = (qf - q0) / (tf - tc); % Maximum velocity (1 x n)

    % Initialize output matrices
    q = zeros(m, n);   % Positions
    qd = zeros(m, n);  % Velocities
    qdd = zeros(m, n); % Accelerations

    % Loop through each joint
    for joint = 1:n
        for i = 1:m
            if t(i) <= tc
                % Acceleration phase
                q(i, joint) = q0(joint) + spp(joint) * t(i)^2 / (2 * tc);
                qd(i, joint) = spp(joint) * t(i) / tc;
                qdd(i, joint) = spp(joint) / tc;
            elseif t(i) <= (tf - tc)
                % Constant velocity phase
                q(i, joint) = q0(joint) + spp(joint) * (t(i) - tc / 2);
                qd(i, joint) = spp(joint);
                qdd(i, joint) = 0;
            elseif t(i) <= tf
                % Deceleration phase
                q(i, joint) = qf(joint) - spp(joint) * (tf - t(i))^2 / (2 * tc);
                qd(i, joint) = spp(joint) * (tf - t(i)) / tc;
                qdd(i, joint) = -spp(joint) / tc;
            end
        end
    end
end
