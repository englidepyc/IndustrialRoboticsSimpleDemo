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

% Define joint variables (initial positions)
t1 = 0; t2 = 0; t3 = 0; t4 = 0; t5 = 0; t6 = 0;

% DH-Parameter
DH = [0  alpha1 d1  t1;
      a2 0      0  t2;
      0  alpha3 0  t3;
      0 -alpha4 d4  t4;
      0  alpha5 0  t5;
      0  0      d6 t6];

% Define basic transformation
Tbase = [1  0  0  0; 
         0  1  0  0;
         0  0  1  0;  
         0  0  0  1];

% Create the links
L1 = Link('d', d1, 'a', 0, 'alpha', alpha1); % revolute 1
L2 = Link('d', 0, 'a', a2, 'alpha', 0); % revolute 2
L3 = Link('d', 0, 'a', 0, 'alpha', alpha3); % revolute 3
L4 = Link('d', d4, 'a', 0, 'alpha', -alpha4); % revolute 4
L5 = Link('d', 0, 'a', 0, 'alpha', alpha5); % revolute 5
L6 = Link('d', d6, 'a', 0, 'alpha', 0); % revolute 6

% Create robot
ourbot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'es.1');
ourbot.base = Tbase;

end