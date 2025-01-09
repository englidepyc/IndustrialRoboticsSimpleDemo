% DIRECT PATH PLANNING OF ANTROPOMORPHIC ARM WITH SPHERICAL WRIST

% Clear environment
clear all;
close all;

%Create antropomorphic arm with spherical wrist
bot = create_bot();


% DEFINE PARAMETERS FOR TRAJECTORY

    % Define start and end positions in joint space
    q_start = [0 0 0 0 0 0]; % starting joint angles
    q_end = [pi/4 -pi/6 pi/3 -pi/4 pi/6 -pi/3]; % ending joint angles
    
    % Define time vector for trajectory
    t = 0:5/99:5; % 100 points over 5 seconds


% GENERATE TRAJECTORY - VALUES FOR Q, QD and QDD
    
    % Generate trajectory with jtraj (5th grade, not exactly trapezodial)
    %[q, qd, qdd] = jtraj(q_start, q_end, t);
    
    % Generate trapezoidal trajectory (with own function)
    [q, qd, qdd] = trapezoidal_trajectory(q_start, q_end, t,5,1); %5 Seconds max time, 1 Second acceleration time

    %Test if the warning about triangular trajectory works
    %[q, qd, qdd] = trapezoidal_trajectory(q_start, q_end, t,5,2.5);


% INITIALIZE ARRAYS FOR FORWARD KINEMATICS

    positions = zeros(length(t), 3); % store [x, y, z]
    orientations = zeros(length(t), 3); % store [roll, pitch, yaw]
    end_effector_velocities = zeros(length(t), 3); % store [vx, vy, vz]
    end_effector_accelerations = zeros(length(t), 3); % store [ax, ay, az]


% COMPUTE FORWARD KINEMATICS, VELOCITIES AND ACCELERATIONS AT EACH POINT IN TRAJECTORY    

    for i = 1:length(t) %for every time step
        
        % Get the forward kinematics for the current joint configuration
        T = bot.fkine(q(i, :)); 
        positions(i, :) = transl(T); % extract position

        %rpy = tr2rpy(T); % extract roll-pitch-yaw orientation
        rpy = tr2rpy_own_try(T.T); % Own version of tr2rpy function, Since T is a Serial Link Element we have to extract only the transformationmatrix with T.T
    
        orientations(i, :) = rpy; % store orientation
    
        % Jacobian for velocity calculation
        J = bot.jacob0(q(i, :)); % compute Jacobian matrix
        end_effector_velocities(i, :) = J(1:3, :) * qd(i, :)'; % velocity of end-effector
    
        % Acceleration calculation (using the Jacobian)
        end_effector_accelerations(i, :) = J(1:3, :) * qdd(i, :)'; % acceleration of end-effector
    end


%PLOTTING OF RESULTS

    % Plot joint space trajectories (position, velocity, and acceleration)
    figure;
    subplot(3,1,1);
    plot(t, q);
    xlabel('Time (s)');
    ylabel('Joint Angles (rad)');
    title('Joint Position Trajectories');
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
    
    subplot(3,1,2);
    plot(t, qd);
    xlabel('Time (s)');
    ylabel('Joint Velocities (rad/s)');
    title('Joint Velocity Trajectories');
    legend('qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6');
    
    subplot(3,1,3);
    plot(t, qdd);
    xlabel('Time (s)');
    ylabel('Joint Accelerations (rad/s²)');
    title('Joint Acceleration Trajectories');
    legend('qdd1', 'qdd2', 'qdd3', 'qdd4', 'qdd5', 'qdd6');
    
    % Plot end-effector position, velocity, and acceleration
    figure;
    subplot(3,1,1);
    plot(t, positions);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('End-Effector Position');
    legend('x', 'y', 'z');
    
    subplot(3,1,2);
    plot(t, end_effector_velocities);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('End-Effector Velocity');
    legend('vx', 'vy', 'vz');
    
    subplot(3,1,3);
    plot(t, end_effector_accelerations);
    xlabel('Time (s)');
    ylabel('Acceleration (m/s²)');
    title('End-Effector Acceleration');
    legend('ax', 'ay', 'az');
    
    % Plot 3D trajectory of the end-effector
    figure;
    plot3(positions(:,1), positions(:,2), positions(:,3), 'LineWidth', 2);
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('End-Effector Trajectory in 3D Space');
    
    % Plot the rpy angles
    figure;
    plot(t,orientations);
    grid on;
    legend('r','p','y');
    title('Rpy over time');


    % Plot robot movement
    %bot.plot(q);
