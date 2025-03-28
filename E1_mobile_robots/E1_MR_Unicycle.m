clc
clear
close all

%% Custom Unicycle Kinematic Model
tspan = 0:0.05:1; 
initialState = [0 0 0];
[t, q] = ode45(@unicycle, tspan, initialState);

figure
plot(q(:,1), q(:,2), 'LineWidth', 2)
title('Trajectory - Custom Unicycle Kinematic Model')
xlabel('X Position (m)')
ylabel('Y Position (m)')
grid on

%% Using MATLAB''s unicycleKinematics
input = [0.2, pi/4];
unicycleModel = unicycleKinematics("VehicleInputs", "VehicleSpeedHeadingRate");
[t, q_unicycle] = ode45(@(t, q_unicycle) derivative(unicycleModel, q_unicycle, input), tspan, initialState);

figure
plot(q_unicycle(:,1), q_unicycle(:,2), 'LineWidth', 2)
title('Trajectory - MATLAB Built-In Unicycle Kinematics')
xlabel('X Position (m)')
ylabel('Y Position (m)')
grid on

%% Custom Unicycle Kinematic Model Function
function dq = unicycle(t, q)
    % Extract state variables
    x  = q(1);
    y  = q(2);
    th = q(3);

    % Model parameters
    r    = 0.1;  % Radius of the wheel (m)
    dph  = 2;    % Wheel speed (rad/s)

    % Compute velocities
    v = r * dph;   % Linear velocity (m/s)
    w = pi/4;      % Angular velocity (rad/s)

    % Compute state derivatives
    dq = [cos(th) 0; 
          sin(th) 0; 
          0       1] * [v; w];
end