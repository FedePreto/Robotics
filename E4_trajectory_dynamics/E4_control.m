%% Decentralized control: Joint2 Puma560

clc
clear 
close all

%Joint2 motor parameters
Km = 0.228; % [N*m*A^-1] Motor Torque constant
Jm = 200e-6; % [kg*m^2] Motor inertia
Bm = 817e-6; % [N*m*s*rad^-1] Motor & joint viscous friction
G = 107.815; % Motor gear ratio
tau_max = 0.9; % [N*m] maximum torque
qd_max = 165; % [rad/s] max speed

G = tf([Km],[Jm Bm]); %first order model of the motor
rltool(G)

Kp = 0.0032121;
Ki = 6.716*Kp;

s = tf('s');
C = (Kp*(s+Ki/Kp))/s;