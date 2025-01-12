% Initialize the Robotics Toolbox
clear; clc;

% Define the 2-DOF planar arm
L1 = Link('d', 0, 'a', 1, 'alpha', 0);  % First link
L2 = Link('d', 0, 'a', 1, 'alpha', 0);  % Second link

% Create the serial link robot
robot = SerialLink([L1 L2], 'name', '2-DOF Planar Arm');

% Define a configuration for the robot
theta1 = pi/4;  % Joint angle 1
theta2 = pi/4;  % Joint angle 2
q = [theta1 theta2];  % Joint angles

% Compute the Jacobian at the given configuration
J = robot.jacob0(q);  % Spatial Jacobian

% Extract the linear velocity part (first two rows for a planar robot)
Jv = J(1:2, 1:2);

% Compute the manipulability ellipsoid (based on the Jacobian)
W = Jv * Jv';  % Ellipsoid matrix

% Generate points for the ellipsoid
theta = linspace(0, 2*pi, 100);  % Parameter for ellipse
ellipse_points = [cos(theta); sin(theta)];  % Unit circle

% Transform unit circle using the square root of W
[v, d] = eig(W);  % Eigen-decomposition
ellipsoid_transformed = v * sqrt(d) * ellipse_points;

% Plot the manipulability ellipsoid
figure;
hold on;
axis equal;
grid on;

% Plot the ellipsoid
plot(ellipsoid_transformed(1, :), ellipsoid_transformed(2, :), 'r-', 'LineWidth', 2);

% Add robot for reference
robot.plot(q, 'workspace', [-2 2 -2 2 -0.5 0.5]);
title('Manipulability Ellipsoid of a 2-DOF Planar Arm');
xlabel('X-axis');
ylabel('Y-axis');
hold off;