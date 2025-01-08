%General structure for inverse kinematic
clear all;
close all;

%Plotting the robot at home and the point to reach
bot = create_bot();
bot.plot([0 0 0 0 0 0]);  % Plot the robot in its home position
hold on;  % Keep the robot plot active
%chosen point (xyz) and making the rotation matrix
plot3(10, -10, 10, 'ro', 'MarkerFaceColor', 'r');  % Red point in space
hold off;

%Choosing point (10, -10, 10) and unit rotation
Target_t = [1, 0, 0, 10;
            0, 1, 0, -10;
            0, 0, 1, 10;
            0, 0, 0, 1]

%Compute the inverse kinematic of the chosen point
target_q = bot.ikine(Target_t, 'mask', [1 1 1 1 1 1]);

pause();

%Plotting the robot at the final point
bot.plot(target_q); 
hold on;  % Keep the robot plot active
%chosen point (xyz) and making the rotation matrix
plot3(10, -10, 10, 'ro', 'MarkerFaceColor', 'r');  % Red point in space
hold off;



%%define final position and orientation of Endeffector
%target_pos = [x, y, z];
%target_rot = R; %rotational matrix
%T_target = [target_rot, target pos'; 0, 0, 0, 1];

%define starting joint configuration
%q0 = [0 0 0 0 0 0];


%use numerical method to find the joint configuration
%q_target = Antropomorphic.ikine(T_target, q0);
