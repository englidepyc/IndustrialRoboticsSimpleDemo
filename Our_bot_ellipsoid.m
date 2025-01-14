clear; close;

robot = create_bot();

% Define a configuration for the robot
q_rotational_singularity = [0,0,0,0,0,0];
q_boundary_singularity = [0,pi/2,pi/2,0,0,0]; %translational singularity
q_nonsingular = [-pi,pi/6,pi/3,-pi/4,pi/6,-pi/2];
q = q_nonsingular;

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

%Display values 
w_q = sqrt(det(J*(J.')));
sv_x = sqrt(Dv(1,1)); sv_y = sqrt(Dv(2,2)); sv_z = sqrt(Dv(3,3));
so_x = sqrt(Do(1,1)); so_y = sqrt(Do(2,2)); so_z = sqrt(Do(3,3));

fprintf("The manipulability index w(q) is: %d\n",w_q);
fprintf("The singular values for σ₁ σ₂ σ₃ in translation are: %d, %d, %d\n",sv_x,sv_y,sv_z);
fprintf("The singular values for σ₁ σ₂ σ₃ in translation are: %d, %d, %d\n",so_x,so_y,so_z);




% Plot the robot and the ellipsoid
figure;
hold on;
grid on;
axis equal;
zlim([-20, 35]);

% Plot the robot
robot.plot(q,'workspace',[-30,30,-20,20,-20,20]);

%Plot the ellipsoid
create_and_plot_man_ellipsoid(Wv,Pe,'y');
create_and_plot_man_ellipsoid(Wo,Pe,'b');


% Label axes
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Manipulability Ellipsoid of a 6-DOF Robot Arm');


hold off;
