clear; clc;

robot = create_bot();

% Define a configuration for the robot
q = [0,0,0,0,pi/2,0];  % Joint angles

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
robot.plot(q);

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
