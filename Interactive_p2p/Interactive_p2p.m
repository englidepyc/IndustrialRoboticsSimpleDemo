clear all; close all;       %Later: show axes of the configuration
home = zeros(1,6);
current_q = home;
pose_reachable = false;
target_q = zeros(1,6);
target_T = zeros(4,4);
pose = zeros(1,6);

%spawn robot
bot = create_bot();

%input a pose
while ~pose_reachable

    pose = input_pose();
    target_T = poserpy2t(pose);
    
    %check if the point is in the workspace
    try
        target_q = bot.ikine(target_T,'mask',ones(1,6));
    catch
        frpintf("The desired pose is out of the reachable workspace, try again \n");
    end
    pose_reachable = true;
end

%SETTING UP THE JOINT TRAPEZOIDAL TRAJECTOR TO REACH THE POINT % can do interactively
t = 0:1/10:5; %in 5 seconds with a resolution of a tenth of a second
tf = 5;
tc = 1; %trying the acceleration phase
[q, qd, qdd] = trapezoidal_trajectory(current_q, target_q, t, tf, tc);

% PLOTTING THE MOVEMENT OF THE END EFFECTOR ON THE JOINT TRAJECTORY
plot3(pose(1,1),pose(1,2), pose(1,3), 'ro', 'MarkerFaceColor', 'r');
bot.plot(q);




