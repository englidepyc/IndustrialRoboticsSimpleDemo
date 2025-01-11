%modified direct path planning script
% added different time vector definition
% testing of own trapezodial function and tr2rpy function
% added figures of joint and endeffector position, velocity and
% acceleration



% Clear environment
clear all;
close all;

% Define parameters
alpha1 = pi/2;
alpha3 = pi/2;
alpha4 = pi/2;
alpha5 = pi/2;
d1 = 6;
d4 = 7;
d6 = 3;
a2 = 8;

% Define joint variables (initial positions)
t1 = 0; t2 = 0; t3 = 0; t4 = 0; t5 = 0; t6 = 0;

% DH-Parameter
DH = [0  alpha1 d1  t1;
      a2 0      0  t2;
      0  alpha3 0  t3;
      0 -alpha4 d4  t4;
      0  alpha5 0  t5;
      0  0      d6 t6];

% Define basic transformation
Tbase = [1  0  0  0; 
         0  1  0  0;
         0  0  1  0;  
         0  0  0  1];

% Create the links
L1 = Link('d', d1, 'a', 0, 'alpha', alpha1); % revolute 1
L2 = Link('d', 0, 'a', a2, 'alpha', 0); % revolute 2
L3 = Link('d', 0, 'a', 0, 'alpha', alpha3); % revolute 3
L4 = Link('d', d4, 'a', 0, 'alpha', -alpha4); % revolute 4
L5 = Link('d', 0, 'a', 0, 'alpha', alpha5); % revolute 5
L6 = Link('d', d6, 'a', 0, 'alpha', 0); % revolute 6

% Create robot
ourbot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'es.1');
ourbot.base = Tbase;

% Define start and end positions in joint space
q_start = [0 0 0 0 0 0]; % starting joint angles
q_end = [pi/4 -pi/6 pi/3 -pi/4 pi/6 -pi/3]; % ending joint angles


% Define time vector for trajectory
t = 0:5/99:5; % 100 points over 5 seconds

% Generate trapezoidal trajectory
%[q, qd, qdd] = jtraj(q_start, q_end, t);

%Test own trapez.-function
[q, qd, qdd] = trapezoidal_trajectory(q_start, q_end, t,5,1); %5 Seconds max time, 1 Second acceleration time

% Initialize arrays for forward kinematics
positions = zeros(length(t), 3); % store [x, y, z]
orientations = zeros(length(t), 3); % store [roll, pitch, yaw]
end_effector_velocities = zeros(length(t), 3); % store [vx, vy, vz]
end_effector_accelerations = zeros(length(t), 3); % store [ax, ay, az]

% Compute forward kinematics, velocities, and accelerations at each point in trajectory
for i = 1:length(t)
    % Get the forward kinematics for the current joint configuration
    T = ourbot.fkine(q(i, :)); % forward kinematics
    positions(i, :) = transl(T); % extract position
    %rpy = tr2rpy(T); % extract roll-pitch-yaw orientation
    rpy = tr2rpy_own_try(T.T); %Own version of tr2rpy function, Since T is a Serial Link Element we have to extract only the transformationmatrix with T.T

    orientations(i, :) = rpy; % store orientation
    
    % Jacobian for velocity calculation
    J = ourbot.jacob0(q(i, :)); % Jacobian matrix
    end_effector_velocities(i, :) = J(1:3, :) * qd(i, :)'; % velocity of end-effector
    
    % Acceleration calculation (using the Jacobian)
    end_effector_accelerations(i, :) = J(1:3, :) * qdd(i, :)'; % acceleration of end-effector
end

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
