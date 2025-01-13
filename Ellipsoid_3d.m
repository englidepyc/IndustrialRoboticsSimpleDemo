% Initialize the Robotics Toolbox
clear; clc;

% Define the 6-DOF arm using Denavit-Hartenberg parameters
L1 = Link('d', 0.5, 'a', 0, 'alpha', pi/2);  % Link 1
L2 = Link('d', 0, 'a', 0.5, 'alpha', 0);    % Link 2
L3 = Link('d', 0, 'a', 0.5, 'alpha', 0);    % Link 3
L4 = Link('d', 0.5, 'a', 0, 'alpha', pi/2); % Link 4
L5 = Link('d', 0, 'a', 0, 'alpha', -pi/2);  % Link 5
L6 = Link('d', 0.2, 'a', 0, 'alpha', 0);    % Link 6

% Create the serial link robot
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '6-DOF Arm');

% Define a configuration for the robot
q = [pi/6, pi/4, -pi/3, pi/4, -pi/6, pi/3];  % Joint angles

%compute the end effector position
Pe = transl(robot.fkine(q));

% Compute the Jacobian at the given configuration
J = robot.jacob0(q);  % Spatial Jacobian

% Extract the translational part of the Jacobian
Jv = J(1:3, 1:6);  % First three rows correspond to translational velocity

% Compute the manipulability ellipsoid (based on the Jacobian)
W = Jv * Jv';  % Ellipsoid matrix

% Generate points for the ellipsoid
[U, D] = eig(W);  % Eigen-decomposition of W

% Create the ellipsoid points
[ellipsoid_x, ellipsoid_y, ellipsoid_z] = ellipsoid(0, 0, 0, sqrt(D(1,1)), sqrt(D(2,2)), sqrt(D(3,3)));

% Transform the ellipsoid based on eigenvectors
ellipsoid_points = [ellipsoid_x(:), ellipsoid_y(:), ellipsoid_z(:)] * U';
ellipsoid_x_transformed = reshape(ellipsoid_points(:, 1), size(ellipsoid_x));
ellipsoid_y_transformed = reshape(ellipsoid_points(:, 2), size(ellipsoid_y));
ellipsoid_z_transformed = reshape(ellipsoid_points(:, 3), size(ellipsoid_z));

% Plot the robot and the ellipsoid
figure;
hold on;
grid on;
axis equal;

% Plot the robot
robot.plot(q, 'workspace', [-1.5 1.5 -1.5 1.5 -1 2]);

% Plot the manipulability ellipsoid
surf(ellipsoid_x_transformed, ellipsoid_y_transformed, ellipsoid_z_transformed, ...
    'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'r');
%Plotting the fkine
plot3(Pe(1), Pe(2), Pe(3),'ro', 'MarkerFaceColor', 'g');

% Label axes
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Manipulability Ellipsoid of a 6-DOF Robot Arm');
hold off;
