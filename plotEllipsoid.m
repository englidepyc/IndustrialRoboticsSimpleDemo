function plotEllipsoid ()
    %J = bot.jacob0(zeros(1,6));
% Import the Robotics Toolbox
% (Ensure it's installed and added to MATLAB path)

% Define a simple 2-DOF robot using DH parameters
L1 = Link([0 0 1 0], 'name', 'L1');
L2 = Link([0 0 1 0], 'name', 'L2');
robot = SerialLink([L1 L2], 'name', '2-DOF Robot');

% Display the robot's home configuration
robot.plot([0 0]);

% Set the joint angles where you want to compute the manipulability
q = [0, pi/2];  % Example joint configuration (in radians)

% Compute the Jacobian matrix at the current joint configuration
J = robot.jacob0(q);
disp(rank(J))
% Compute the manipulability matrix W = J * J'
W = J * J';  % W is a positive semi-definite matrix

% Perform Singular Value Decomposition (SVD) to get the singular values
[U, S, V] = svd(W);

% Get the singular values (diagonal of S)
singular_values = diag(S);  % These are the singular values (scalars)

% Create a unit sphere (for scaling purposes)
[U_sphere, V_sphere] = sphere(20);  % Parametric sphere

% Scale the unit sphere based on the singular values
ellipsoid = U_sphere * singular_values(1) + V_sphere * singular_values(2);

% Plot the manipulability ellipsoid
figure;
hold on;
plot3(ellipsoid(1,:), ellipsoid(2,:), ellipsoid(3,:), 'r');
title('Manipulability Ellipsoid');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
grid on;


end