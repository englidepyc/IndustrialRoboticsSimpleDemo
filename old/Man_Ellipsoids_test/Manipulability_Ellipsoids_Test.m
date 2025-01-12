%Test for manipulability
clear all;
close all;

q = [0 0 0 0 0 0];
%q = [0 pi/3 0 0 0 pi/7]; %add values to test
robot = create_bot();

%Calculation of Jacobian
J = robot.jacob0(q);

%calculation of singular values
%singular_values = svd(J) %easy way

J_Jt = J*J.'
eigenvalues = eig(J_Jt);
singular_values = sqrt(eigenvalues)

%Volume of ellipsoid
%w = prod(singular_values)
w = sqrt(det(J_Jt))

%plot ellipsoid
figure;
axis equal;
[X, Y, Z] = ellipsoid(0, 0, 0, singular_values(1), singular_values(2), singular_values(3), 20);
surf(X, Y, Z, 'FaceAlpha', 0.5);

