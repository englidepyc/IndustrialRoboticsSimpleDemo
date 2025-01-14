% Clear environment
clear all;
close all;

% Create bot
bot = create_bot();

% Define start and end positions in joint space
q_start = [0 0 0 0 0 0]; % starting joint angles
q_end = [-pi pi/6 pi/3 -pi/4 pi/6 -pi/2]; % ending joint angles

% Define time vector for trajectory
t = 0:1/10:5; 
tc = 3.0; % acceleration time

% Creating trapzodial trajectory
[q, qd, qdd] = trapezoidal_trajectory(q_start, q_end, t,tc); 

% Initialize array for plotting the position
positions = zeros(length(t), 3); % store [x, y, z]

% Create a figure for plotting
figure;
hold on; % keep all the plots on the same figure
grid on;
axis equal;
zlim([-24, 15]);

xlabel('X');
ylabel('Y');
zlabel('Z');

% Create video object
video_filename = 'p2p_dir_kin_trapezodial.mp4'; 
video_obj = VideoWriter(video_filename, 'MPEG-4'); 
video_obj.FrameRate = 10; % set frame rate to 10 fps
open(video_obj); % open video object

% Compute forward kinematics, velocities, and accelerations at each point in trajectory
for i = 1:length(t)

    % Get the forward kinematics for the current joint configuration
    T = bot.fkine(q(i, :)); % forward kinematics
    positions(i, :) = transl(T); % extract position (x y z)

    % Plot the robot at the current joint configuration
    bot.plot(q(i, :)); 
    plot3(positions(i,1), positions(i,2), positions(i,3), '.r'); 
    % red dot for every trajectory point
    
    % Save current frame
    frame = getframe(gcf); 
    writeVideo(video_obj, frame); 
end

% Close video object and safe file
close(video_obj); 


% Plot joint space trajectories (position, velocity, and acceleration)
%...


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





