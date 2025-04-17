clc
clear
close all

[puma,conf] = loadrvcrobot("puma"); %Puma560
puma.show(conf.qn);

qd = zeros(1,6);
Q = puma.inverseDynamics(conf.qn,qd,qd) % Inverse dynamics to mantain the robot in a static position
% The 2 joint that put more effort are the second and the third, this is 
% because they have to keep in place all the structure 

%We can print the characteristic of the single part of the robot
puma.Bodies{1};
puma.Bodies{2};

% If we decrease the contribution of the gravity we can see that the joint
% require less effort

puma.Gravity = puma.Gravity/6;
%Q = puma.gravityTorque(conf.qn)
Q = puma.inverseDynamics(conf.qn,qd,qd)

%% Inertia (Mass) matrix
M = puma.massMatrix(conf.qn)

%% Coriolis matrix
qd = [0 0 1 0 0 0];
C = coriolisMatrix(puma,conf.qn,qd) 
vp = puma.velocityProduct(conf.qn,qd); %C(q,qd)*qd   

%% Forward dynamics

ur5 = loadrobot("universalUR5e","DataFormat","row","Gravity",[0 0 -9.81]);
ur5conf.qn = [0 -1.07 1.38 -0.3 0 -2.3];
qdd = ur5.forwardDynamics(ur5conf.qn)

%% Exploring a model under zero torque, just gravity effect 
sl_zerotorque
r = sim("sl_zerotorque");
t = r.find("tout");
q = r.find("yout");
plot(t,q(:,1:3))

rc = rateControl(10);
for i = 1:size(q,1)
    ur5.show(q(i,:),FastUpdate=true,PreservePlot=false);
    rc.waitfor;
end

