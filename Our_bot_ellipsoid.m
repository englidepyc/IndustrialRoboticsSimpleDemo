clear; clc;

robot = create_bot();

% Define a configuration for the robot
q_1 = [-pi,pi/6,pi/3,-pi/4,pi/6,-pi/2];
q = q_1;

%compute the end effector position
Pe = transl(robot.fkine(q));

% Compute the Jacobian at the given configuration
J = robot.jacob0(q);  % Spatial Jacobian

% Extract Jv and Jo
Jv = J(1:3, 1:6); 
Jo = J(4:6, 1:6);

% Compute the manipulability matrix (based on the Jacobian)
Wo = Jo * Jo';
Wv = Jv * Jv';  

% Calculate eigenvalues and vectors
[Uv, Dv] = eig(Wv);
[Uo, Do] = eig(Wo);

%Display the w(q)
w_q = sqrt(det(J*(J.'))); fprintf("The manipulability index w(q) is %d \n", w_q);

% Plot the robot and the ellipsoid
figure;
hold on;
grid on;
axis equal;

% Plot the robot
robot.plot(q);

%Plot the ellipsoid
create_and_plot_man_ellipsoid(Wv,Pe,'r');
create_and_plot_man_ellipsoid(Wo,Pe,'g');


% Label axes
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Manipulability Ellipsoid of a 6-DOF Robot Arm');
hold off;
