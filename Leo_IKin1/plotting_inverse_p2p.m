clear all;
close all;

q0 = zeros(1,6) %home
%CHOOSING THE POINT (10, -10, 10) and unit rotation
Target_t = [1, 0, 0, 10;
            0, 1, 0, -10;
            0, 0, 1, 10;
            0, 0, 0, 1]

%COMPUTING THE INVERSE KINEMATICS OF THE CHOSEN POINT
bot = create_bot();
target_q = bot.ikine(Target_t, 'mask', [1 1 1 1 1 1]);

%SETTING UP THE JOINT TRAPEZOIDAL TRAJECTOR TO REACH THE POINT
t = 0:1/10:5;
tf = 5;
tc = 1; %trying the acceleration phase
[q, qd, qdd] = trapezoidal_trajectory(q0, target_q, t, tf, tc);

%% PLOTTING THE MOVEMENT OF THE END EFFECTOR ON THE JOINT TRAJECTORY
plot3(10, -10, 10, 'ro', 'MarkerFaceColor', 'r');
bot.plot(q);