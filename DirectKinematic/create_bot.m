function ourbot = create_bot()

% Define parameters
alpha1 = pi/2;
alpha3 = pi/2;
alpha4 = pi/2;
alpha5 = pi/2;
d1 = 6;
d4 = 7;
d6 = 3;
a2 = 8;

% Define basic transformation
Tbase = eye(4);

% Create the links
L1 = Link('d', d1, 'a', 0, 'alpha', alpha1);  % revolute 1
L2 = Link('d', 0, 'a', a2, 'alpha', 0);       % revolute 2
L3 = Link('d', 0, 'a', 0, 'alpha', alpha3);   % revolute 3
L4 = Link('d', d4, 'a', 0, 'alpha', -alpha4); % revolute 4
L5 = Link('d', 0, 'a', 0, 'alpha', alpha5);   % revolute 5
L6 = Link('d', d6, 'a', 0, 'alpha', 0);       % revolute 6

% Create robot
ourbot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', ' ');
ourbot.base = Tbase;

end


