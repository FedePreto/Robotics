clc
clear
close all

%% Differential Drive model with differential equations
tspan = 0:0.05:1; 
initialState = [0 0 0];
[t,q] = ode45(@diffDrive,tspan,initialState);

figure
plot(q(:,1),q(:,2))


%% Equivalently I can diffDriveKinematics

input = [0.5, -2];
diffDriveModel = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");
diffDriveModel.TrackWidth = 0.5;
diffDriveModel.WheelRadius = 0.1;
[t, q_diffDrive] = ode45(@(t,q_diffDrive)derivative(diffDriveModel,q_diffDrive,input),tspan,initialState);

figure
plot(q_diffDrive(:,1),q_diffDrive(:,2))

function dq = diffDrive(t,q)
    x = q(1);
    y = q(2);
    th = q(3);

    r = 0.1; %radius of the wheel
    d = 0.5; %Distance between the back wheels
    
    dph_L = 10; % Left wheel speed
    dph_R = -10; % Right wheel speed

    v = r/2 * (dph_R + dph_L); %heading linear speed
    w = r/d * (dph_R - dph_L); %heading angular speed

    dq = [cos(th) 0; sin(th) 0; 0 1]*[v; w];
end