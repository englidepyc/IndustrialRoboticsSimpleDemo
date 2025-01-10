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
Antropomorphic1 = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'es.1');
Antropomorphic1.base = Tbase;

% Define start and end positions in joint space
q_start = [0 0 0 0 0 0]; % starting joint angles
q_end = [pi/4 -pi/6 pi/3 -pi/4 pi/6 -pi/3]; % ending joint angles

% Define time vector for trajectory
t = linspace(0, 5, 100); % 100 points over 5 seconds


% Generate trapezoidal trajectory
[q, qd, qdd] = jtraj(q_start, q_end, t);

% Initialize arrays for forward kinematics
positions = zeros(length(t), 3); % store [x, y, z]
orientations = zeros(length(t), 3); % store [roll, pitch, yaw]

% Compute forward kinematics at each point in trajectory
for i = 1:length(t)
    T = Antropomorphic1.fkine(q(i, :)); % forward kinematics
    positions(i, :) = transl(T); % extract position
    rpy = tr2rpy(T); % extract roll-pitch-yaw orientation
    orientations(i, :) = rpy; % store orientation
end

% Plotting q %
figure;
subplot(2,1,1);
plot(t, q);
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
title('Joint Space Trajectories');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');

% Plot direct kinematics (end-effector position)
subplot(2,1,2);
plot(t, positions);
xlabel('Time (s)');
ylabel('Position (m)');
title('End-Effector Position');
legend('x', 'y', 'z');

% Visualize end-effector trajectory in 3D space
figure;
plot3(positions(:,1), positions(:,2), positions(:,3), 'LineWidth', 2);
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('End-Effector Trajectory in 3D Space');

%Plotting orientations (Plotting in 3d gives a similar graph to the "2d" plotting)
figure;
grid on;
plot(t,orientations);
legend('r','p','y');
title('RPY over time');
