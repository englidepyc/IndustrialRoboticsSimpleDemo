function pose = input_pose()
    pose = zeros(1,6);
    fprintf("Enter desired pose:\n");
    pose(1) = input("x: ");
    pose(2) = input("y: ");
    pose(3) = input("z: ");
    pose(4) = input("r: ");
    pose(5) = input("p: ");
    pose(6) = input("y: ");
end