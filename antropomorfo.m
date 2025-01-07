clearvars;
%scrivo i parametri
    alpha_1 = -pi/2;
    d1 = 0.5;
    %teta_1 giunto
    a2 = 0.5;
    %teta_2 giunto
    %teta_2 giunto
    teta_1 = 0; teta_2 = 0; teta_3 = 0;
    DH = [0,alpha_1,d1,teta_1;
        a2,0,0,teta_2;
        0,0,0,teta_3] % potrei spostare il link 3 (a3) per visualizzarlo meglio

%creo robot


L1 = Link('a',0,'alpha',alpha_1,'d',d1,'revolute');
L2 = Link('a',a2,'alpha',0,'d',0,'revolute');
L3 = Link('a',0,'alpha',0,'d',0,'revolute'); 

MyAntro = SerialLink([L1 L2 L3],'name','MyAntro_es1');
%eventuale tbase per ruotare la base
Tbase=[0  1  0 0; 
       -1 0  0 0;
       0  0 1 0;
       0  0 0 1];
MyAntro.base = Tbase;
MyAntro.teach()

%plotto per andamenti arbitrari dei giunti
t = 1:1/25:10;
teta_1 = sin(0.3*t);
teta_2 = cos(t);
teta_3 = 0.3*t;


Q = [teta_1;teta_2;teta_3];

MyAntro.plot(Q','fps',25,'movie','MyAntroMovie.mp4');

%you can plot Euler angles of the end effector