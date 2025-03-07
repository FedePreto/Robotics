clc 
clear
close all

tspan = 0:0.05:1; 
initialState = [0 0 0];

%% Part 1: Custom Bicycle Kinematic Model
[t, q] = ode45(@bicycle, tspan, initialState);

figure
plot(q(:,1), q(:,2), 'LineWidth', 2)
title('Trajectory - Custom Bicycle Kinematic Model')
xlabel('X Position (m)')
ylabel('Y Position (m)')
grid on

%% Part 2: Using MATLAB's bicycleKinematics
input = [2, pi/4];
bicycleModel = bicycleKinematics("VehicleInputs", "VehicleSpeedHeadingRate");
bicycleModel.WheelBase = 0.1;
[t, q_bicycle] = ode45(@(t, q_bicycle) derivative(bicycleModel, q_bicycle, input), tspan, initialState);

figure
plot(q_bicycle(:,1), q_bicycle(:,2), 'LineWidth', 2)
title('Trajectory - MATLAB Built-In Bicycle Kinematics')
xlabel('X Position (m)')
ylabel('Y Position (m)')
grid on

%% Custom Bicycle Kinematic Model Function
function dq = bicycle(t, q)
    % Extract state variables
    x  = q(1);
    y  = q(2);
    th = q(3);

    % Model parameters
    L  = 0.1;   % WheelBase: Distance between the wheels (m)
    ph = 0.5;   % Steering angle (rad) (limited due to physical constraints)

    % Velocities
    v = 1;  % Linear velocity (m/s)
    w = v * tan(ph) / L;  % Angular velocity (rad/s)

    % Compute state derivatives
    dq = [cos(th) 0; 
          sin(th) 0; 
          0       1] * [v; w];
end

