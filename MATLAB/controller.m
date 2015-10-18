function main

clc
clear
close all
    
T = [1 20];

x0 = [0 0]';

[T,Y] = ode45(@dynamics, T, x0);

plot(T,Y)
legend('pos','vel')


end

function dy = dynamics(t, y)

    kp = 2.41421356;
    kd = 2.2;
    
    a = -kp*(y(1)-1) - kd*y(2)
    dy = [0 1; 0 0]*y + [0;a];

end