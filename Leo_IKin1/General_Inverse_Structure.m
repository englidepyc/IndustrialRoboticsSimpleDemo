%General structure for inverse kinematic

%define final position and orientation of Endeffector
%target_pos = [x, y, z];
%target_rot = R; %rotational matrix

%define starting joint configuration
%q0 = [0 0 0 0 0 0];

%create Robot
%L1 = ...
%...
%L6 = ...


%use numerical method to find the joint configuration
%q_target = Robot.ikine(...);

