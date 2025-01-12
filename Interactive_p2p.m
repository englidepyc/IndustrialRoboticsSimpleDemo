clear all; close all;       
home = [0, 0, 0, 0, pi/2, 0];
current_q = home;
target_T = zeros(4,4);
pose = [10,10,10,0,0,0];
continue_entering = true;
total_time = 0;
acceleration_time = 0;

%spawn robot
bot = create_bot();
fk = bot.fkine(home);
T = fk.T;
home_pose = transl(T);

%MAIN LOOP, KEEP ON ENTETING POINTS
while continue_entering
    
    %return home
    if current_q ~= home
    t = 0:1/10:8;
    [q, qd, qdd] = trapezoidal_trajectory(current_q, home, t, 2);
    plot3(home_pose(1),home_pose(2), home_pose(3), 'ro', 'MarkerFaceColor', 'g'); %dk why it doesn't plot
    bot.plot(q);
    current_q = home;
    end

target_T = poserpy2t(pose);
target_q = bot.ikine(target_T,'mask',ones(1,6));

%SETTING UP THE JOINT TRAPEZOIDAL TRAJECTOR TO REACH THE POINT % can do interactively
total_time = input("Enter the total time to reach the point ");
acceleration_time = input("Enter the acceleration time of the joint trapezoidal trajectory ");
%resolution of a tenth of a second
t = 0:1/10:total_time;
[q, qd, qdd] = trapezoidal_trajectory(current_q, target_q, t, acceleration_time);

% PLOTTING THE MOVEMENT OF THE END EFFECTOR ON THE JOINT TRAJECTORY
plot3(pose(1),pose(2), pose(3), 'ro', 'MarkerFaceColor', 'r');
bot.plot(q);
current_q = target_q;

choice = input("Enter q to quit, any key to continue ","s");
if choice=='q'; continue_entering = false; end
    
pose_reachable = false;
end

close all; clear all;



