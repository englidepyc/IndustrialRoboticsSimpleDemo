function T = poserpy2t (pose)
    roll = pose(1,4);
    pitch = pose(1,5);
    yaw = pose(1,6);
    position = pose (1,1:3);
    T = zeros(4,4); T(4,4) = 1;
    T(1:3,4) = position;

    if abs(pitch) == pi/2
        fprintf("Warning, the entered rpy values constitute a representation singularity\n");
    end


%Computing direct rotation matrix

% Rotation about the X-axis (Roll)
Rx = [1, 0, 0;
      0, cos(roll), -sin(roll);
      0, sin(roll), cos(roll)];

% Rotation about the Y-axis (Pitch)
Ry = [cos(pitch), 0, sin(pitch);
      0, 1, 0;
      -sin(pitch), 0, cos(pitch)];

% Rotation about the Z-axis (Yaw)
Rz = [cos(yaw), -sin(yaw), 0;
      sin(yaw), cos(yaw), 0;
      0, 0, 1];

% The total rotation matrix is the product of the individual rotations
R = Rz * Ry * Rx;

%Setting it into the homogeneus transformation matrix
T(1:3,1:3) = R


end

