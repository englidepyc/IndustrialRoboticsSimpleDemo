function [rpy] = tr2rpy_own_try(T) %T = transformation matrix

R = T(1:3,1:3); %Extracting only the rotation part

phi =  atan2(R(2, 1), R(1, 1)); 

theta = atan( -R(3,1)/sqrt(R(3,2)^2+R(3,3)^2) ); 

psi = atan( R(3,2)/R(3,3) ); 

rpy = [phi theta psi];

end



