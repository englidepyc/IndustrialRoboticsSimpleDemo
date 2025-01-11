clear all; close all;       %Later: show axes of the configuration
home = zeros(1,6);
current_q = home;
pose_reachable = false;
target_q = zeros(1,6);
target_T = zeros(4,4);
pose = zeros(1,6);
continue_entering = true;
%spawn robot
bot = create_bot();

%MAIN LOOP, KEEP ON ENTETING POINTS
while continue_entering

%input a pose
while ~pose_reachable

    pose = input_pose();
    target_T = poserpy2t(pose);
    %calculate the inverse kinematics
    try
        target_q = bot.ikine(target_T,'mask',ones(1,6));
    catch
        fprintf("inverse kinematics didn't work, try again \n");
    end
    
    %prompt user to try again if the point is outside of reachable
    %workspace
    if ~isempty(target_q)
        pose_reachable = true;
    else
        fprintf("Point is outside of reachable workspace, try again \n");
    end
end

%SETTING UP THE JOINT TRAPEZOIDAL TRAJECTOR TO REACH THE POINT % can do interactively
t = 0:1/10:5; %in 5 seconds with a resolution of a tenth of a second
tf = 5;
tc = 1; %trying the acceleration phase
[q, qd, qdd] = trapezoidal_trajectory(current_q, target_q, t, tf, tc);

% PLOTTING THE MOVEMENT OF THE END EFFECTOR ON THE JOINT TRAJECTORY
plot3(pose(1,1),pose(1,2), pose(1,3), 'ro', 'MarkerFaceColor', 'r');
bot.plot(q);

choice = input("Enter q to quit, any key to continue ","s");
if choice=='q'
    continue_entering = false;
end
    pose_reachable = false;
end

close all;



