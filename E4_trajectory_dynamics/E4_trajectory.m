clc
clear
close all

%% Trajectory in the joit space

abb = loadrobot("abbIrb1600","DataFormat","row");
abbIK = analyticalInverseKinematics(abb);
abbIKFcn = abbIK.generateIKFunction("ikIrb1600"); %Function that compute the IK

start = se3(3,"roty", [0.6 -0.5 0.1]);
goal = se3(2, "rotx", [0.4 0.5 0.1]);

% Compute the qs that allow the robot to be in the start and
% goal position

start_sol = abbIKFcn(start.tform); 
goal_sol = abbIKFcn(goal.tform);

%We take only one of the two solutions 
startgoal_pts = [start_sol(1,:)' goal_sol(1,:)'];

t = 0:0.02:2;
 
% % Generating the trajectories (trapezoidal)
% [q,qd,qdd] = trapveltraj(startgoal_pts,numel(t),EndTime=2)
% figure("Name","Trapezoidal velocity profile")
% subplot(2,2,1) 
% plot(t,q')
% grid on;
% title("Position $q$","Interpreter","latex")
% 
% subplot(2,2,2)
% plot(t,qd')
% title("Velocity $\dot{q}$","Interpreter","latex")
% grid on;
% 
% subplot(2,2,3) 
% plot(t,qdd')
% grid on;
% title("Acceleration $\ddot{q}$","Interpreter","latex")

% % Generating the trajectories (cubic)
% [q,qd,qdd] = cubicpolytraj(startgoal_pts,[0 2],t)
% figure("Name","Cubic velocity profile")
% subplot(2,2,1) 
% plot(t,q')
% grid on;
% title("Position $q$","Interpreter","latex")
% 
% subplot(2,2,2)
% plot(t,qd')
% title("Velocity $\dot{q}$","Interpreter","latex")
% grid on;
% 
% subplot(2,2,3) 
% plot(t,qdd')
% grid on;
% title("Acceleration $\ddot{q}$","Interpreter","latex")

% Generating the trajectories (fifth order)
[q,qd,qdd] = quinticpolytraj(startgoal_pts,[0 2],t);
figure("Name","Fifth order velocity profile")

subplot(2,2,1) 
plot(t,q')
grid on;
title("Position $q$","Interpreter","latex")

subplot(2,2,2)
plot(t,qd')
title("Velocity $\dot{q}$","Interpreter","latex")
grid on;

subplot(2,2,3) 
plot(t,qdd')
grid on;
title("Acceleration $\ddot{q}$","Interpreter","latex")

% Plotting the solution of traj on joint-space
r = rateControl(60);
figure('Name',"Animation")
grid on;
for i = 1:size(q,2)
    abb.show(q(:,i)',"FastUpdate",true,"PreservePlot",false);
    traj_T(i) = se3(abb.getTransform(q(:,i)',"tool0"));
    r.waitfor;
end

p =traj_T.trvec;
hold on; plot(p(:,1),p(:,2)), hold off


%% Task-space Trajectory
clc
clear
close all

abb = loadrobot("abbIrb1600","DataFormat","row");
abbIK = analyticalInverseKinematics(abb);
abbIKFcn = abbIK.generateIKFunction("ikIrb1600"); %Function that compute the IK

start = se3(3,"roty", [0.6 -0.5 0.1]);
goal = se3(2, "rotx", [0.4 0.5 0.1]);

% Compute the qs that allow the robot to be in the start and
% goal position

start_sol = abbIKFcn(start.tform); 
goal_sol = abbIKFcn(goal.tform);

%We take only one of the two solutions 
startgoal_pts = [start_sol(1,:)' goal_sol(1,:)'];

t = 0:0.02:2;
%We will use minjerkpolytraj to create a smooth profile that is convinient for mechanical systems  
[s,sd,sdd] = minjerkpolytraj([0 1],[0 2],numel(t));

Ts = transformtraj(start,goal,[0 2],t,TimeScaling=[s;sd;sdd]);
p = Ts.trvec;
figure; plot(p(:,1),p(:,2));

q = ikineTraj(abbIKFcn,Ts);
figure('Name',"Animation")
r = rateControl(50);
for i=1:size(q,1)
    abb.show(q(i,:),FastUpdate=true,PreservePlot=false);
    r.waitfor
end

hold on; plot(p(:,1),p(:,2)); hold off

figure('Name','')
title("Minimum jerk trajectory (position $q$)","Interpreter","latex"); 
xplot(t,q,unwrap=true)



