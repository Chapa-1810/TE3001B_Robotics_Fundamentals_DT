dbstop if error
clear all;
clc;
close all;

% Parametros del motor
tau = 7.570081;
k   = 4.6202081;

% Condiciones iniciales
IC      = [0;0];  

% Intervalo del tiempo
TINT    = 0:0.001:50;

[t,state] = ode45(@(t,state) mysolver(t, state, tau, k),TINT,IC);

omega = state(:,1);
z     = state(:,2);

%Velocidad angular simulada
figure
plot(t,omega)
omegamax = max(omega);
omegamin = min(omega);
axis([t(1,1), t(end,1), omegamin-omegamax*0.1, omegamax*1.1]);
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('DCMotor angular velocity, Kp = 0.5 and Ki = 0.6')

kp = 0.5;
ki = 0.6;
omegad = 7.33;
[ren,~] = size(t);
for index = 1:1:ren
    u(index,1) = kp*(omegad - omega(index,1)) + z(index,1);
end

%Entrada de control simulada
figure
plot(t,u)
umax = max(u);
umin = min(u);
axis([t(1,1), t(end,1), umin-umax*0.1, umax*1.1]);
xlabel('Time [s]')
ylabel('Voltage [volts]')
title('Control Input')
 
function dstatedt = mysolver(t, state, tau, k)
 
    omega = state(1,1);
    z     = state(2,1);
    
    % Controlador u(t)
        kp = 0.5;
        ki = 0.6;
        omegad = 7.33;
        u    = kp*(omegad - omega) + z;
        zdot = ki*(omegad - omega);

    % Ecuación diferencial
    omegadot = -tau*omega + k*u;

    dstatedt = [omegadot; zdot];     
     
    display(t)
 end
