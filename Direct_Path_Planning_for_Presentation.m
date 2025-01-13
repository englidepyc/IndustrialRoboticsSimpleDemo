% Clear environment
clear all;
close all;

% Create bot
bot = create_bot();

% Define start and end positions in joint space
q_start = [0 0 0 0 0 0]; % starting joint angles
%q_end = [-pi pi/6 pi/3 -pi/4 pi/6 -pi/2]; % ending joint angles
q_end = [-pi pi/6 pi/3 -pi/4 pi/5 -pi/2];
% Define time vector for trajectory
t = 0:1/10:5; % 100 points over 5 seconds
tf = 5; % final time
tc = 2; % acceleration time

% Creating trapzodial trajectory
%[q, qd, qdd] = trapezoidal_trajectory(q_start, q_end, t,tc); 
[q, qd, qdd] = trapezoidal_trajectory_with_velocity_limit(q_start, q_end, t,tc,pi/10); 
%[q, qd, qdd] = jtraj(q_start, q_end, t); % trajectory using jtraj

% Initialize arrays for forward kinematics
positions = zeros(length(t), 3); % store [x, y, z]
orientations = zeros(length(t), 3); % store [roll, pitch, yaw]

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

    rpy = transform_tr2rpy(T.T); % Custom tr2rpy function
    % Since T is a Serial Link Element we have to extract only the transformationmatrix with T.T
    orientations(i, :) = rpy; % store orientation (r p y)
    
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


% figure;
% subplot(3,1,1);
% plot(t, q);
% xlabel('Time (s)');
% ylabel('Joint Angles (rad)');
% title('Joint Position Trajectories');
% legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
% 
% subplot(3,1,2);
% plot(t, qd);
% xlabel('Time (s)');
% ylabel('Joint Velocities (rad/s)');
% title('Joint Velocity Trajectories');
% legend('qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6');
% 
% subplot(3,1,3);
% plot(t, qdd);
% xlabel('Time (s)');
% ylabel('Joint Accelerations (rad/s²)');
% title('Joint Acceleration Trajectories');
% legend('qdd1', 'qdd2', 'qdd3', 'qdd4', 'qdd5', 'qdd6');

% Create a figure with multiple subplots
figure;

%Set the font to Century Gothic for the current figure
set(gca, 'FontName', 'Century Gothic'); % Set font for the current axes
set(gcf, 'DefaultAxesFontName', 'Century Gothic'); % Set font for all subplots

% Plot for joint positions
subplot(3,1,1);
plot(t, q, 'LineWidth', 0.8); % Set line width to 0.8
xlabel('Time (s)', 'FontSize', 12); % Time on x-axis
ylabel('$q$ (rad)', 'Interpreter', 'latex', 'FontSize', 12); % q in rad on y-axis
grid on;
set(gca, 'FontSize', 12);

% Plot for joint velocities
subplot(3,1,2);
plot(t, qd, 'LineWidth', 0.8); % Set line width to 0.8
xlabel('Time (s)', 'FontSize', 12); % Time on x-axis
ylabel('$\dot{q}$ (rad/s)', 'Interpreter', 'latex', 'FontSize', 12); % qd in rad/s on y-axis
grid on;
set(gca, 'FontSize', 12);

% Plot for joint accelerations
subplot(3,1,3);
plot(t, qdd, 'LineWidth', 0.8); % Set line width to 0.8
xlabel('Time (s)', 'FontSize', 12); % Time on x-axis
ylabel('$\dot{q}$ (rad/s²)', 'Interpreter', 'latex', 'FontSize', 12); % qdd in rad/s² on y-axis
grid on;
set(gca, 'FontSize', 12);

% Add a single legend outside the plots
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'}, ...
       'Location', 'eastoutside', 'FontSize', 12); % Single legend for all plots

