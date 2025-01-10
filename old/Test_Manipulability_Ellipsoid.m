clear all;
close all;

% Robot definition
alpha1 = pi/2; alpha4 = pi/2; alpha5 = pi/2;
a2 = 8; a3 = 4;

L1 = Link('d', 0, 'a', 0, 'alpha', alpha1, 'qlim', [-pi, pi]); % Revolute joint 1
L2 = Link('d', 0, 'a', a2, 'alpha', 0, 'qlim', [-pi/2, pi/2]); % Revolute joint 2
L3 = Link('d', 0, 'a', a3, 'alpha', 0, 'qlim', [-pi/2, pi/2]); % Revolute joint 3
L4 = Link('d', 0, 'a', 0, 'alpha', -alpha4, 'qlim', [-pi, pi]); % Revolute joint 4
L5 = Link('d', 0, 'a', 0, 'alpha', alpha5, 'qlim', [-pi, pi]); % Revolute joint 5
L6 = Link('d', 0, 'a', 0, 'alpha', 0, 'qlim', [-pi, pi]); % Revolute joint 6

AntropomorphicArm = SerialLink([L1, L2, L3, L4, L5, L6], 'name', 'Antropomorphic Arm');

% Define joint ranges (e.g., all angles between -pi/4 and pi/4)
theta1_range = linspace(-pi/4, pi/4, 5); % Range for joint 1
theta2_range = linspace(-pi/4, pi/4, 5); % Range for joint 2
theta3_range = linspace(-pi/4, pi/4, 5); % Range for joint 3

% Compute and visualize Manipulability Ellipsoids
figure;
AntropomorphicArm.plot([0 0 0 0 0 0]); % Initialize the robot in the plot
hold on;

for theta1 = theta1_range
    for theta2 = theta2_range
        for theta3 = theta3_range
            theta = [theta1, theta2, theta3, 0, 0, 0];
            
            % Compute the Jacobian matrix
            J = AntropomorphicArm.jacob0(theta);
            M = J(1:3, :) * J(1:3, :)'; % Translational part of the Jacobian
            [eigenvec, eigenval] = eig(M); % Compute eigenvalues and eigenvectors
            
            % Generate the ellipsoid
            theta_ellipsoid = linspace(0, 2*pi, 100); % Circle parameterization
            circle = [cos(theta_ellipsoid); sin(theta_ellipsoid); zeros(1, length(theta_ellipsoid))];
            ellipsoid = eigenvec * sqrt(eigenval) * circle; % Scale the circle into the ellipsoid
            
            % Compute the end-effector position
            end_effector_pos = AntropomorphicArm.fkine(theta).t;
            
            % Plot the ellipsoid (shifted to the end-effector position)
            plot3(ellipsoid(1,:) + end_effector_pos(1), ...
                  ellipsoid(2,:) + end_effector_pos(2), ...
                  ellipsoid(3,:) + end_effector_pos(3), 'r', 'LineWidth', 0.5);
        end
    end
end

% Add labels and finalize the plot
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Manipulability Ellipsoids for Multiple Configurations');
axis equal;
grid on;
hold off;
