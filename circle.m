close all; clear; clc;

r = 3;
n = 60*5;

t = linspace(0, 2, n);

x = r*sin(-t);
y = -5+zeros(1, n);
z = r*cos(-t);

animate_cobot_path("Circle", [x; y; z]);