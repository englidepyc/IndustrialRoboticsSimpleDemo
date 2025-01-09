%Test Trapezodial - Triangular
qstart = [0 0]
qend = [1 3];
t = 0:1/10:5;
tf = 5;
tc = 2.5;

[q, qd, qdd] = trapezoidal_trajectory(qstart, qend, t, tf, tc);

%PLOTTING 
% Position (q)
subplot(3, 1, 1);
plot(t, q(:, 1), 'r', 'LineWidth', 2); hold on;
plot(t, q(:, 2), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position (rad)');
title('Joint Position Trajectories');
legend('q1', 'q2');
grid on;

% Velocity (qd)
subplot(3, 1, 2);
plot(t, qd(:, 1), 'r', 'LineWidth', 2); hold on;
plot(t, qd(:, 2), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
title('Joint Velocity Trajectories');
legend('qd1', 'qd2');
grid on;

% Acceleration (qdd)
subplot(3, 1, 3);
plot(t, qdd(:, 1), 'r', 'LineWidth', 2); hold on;
plot(t, qdd(:, 2), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Acceleration (rad/s²)');
title('Joint Acceleration Trajectories');
legend('qdd1', 'qdd2');
grid on;