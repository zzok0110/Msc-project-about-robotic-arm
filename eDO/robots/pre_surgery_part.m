% Project code (Towards Autonomous Surgery: Algorithms)
% Created: June 2023 by Zhe Zhu

%% The preparation process before surgery
% to get ideal joint configurations without self collisions from Cartesian space
% the code includes testing 

clear;
clc;

edo = importrobot('edo_sim.urdf');
edo.Gravity = [0 0 -9.8];
edo.DataFormat = 'row';
show(edo);
% set joint range
edo.Bodies{1, 1}.Joint.PositionLimits = [-inf inf];
edo.Bodies{1, 2}.Joint.PositionLimits = [-pi*11/18 pi*11/18];
edo.Bodies{1, 3}.Joint.PositionLimits = [-pi*11/18 pi*11/18];
edo.Bodies{1, 4}.Joint.PositionLimits = [-inf inf];
edo.Bodies{1, 5}.Joint.PositionLimits = [-pi*11/18 pi*11/18];
edo.Bodies{1, 6}.Joint.PositionLimits = [-inf inf];

start = getTransform(edo,edo.homeConfiguration,"base_link","link_6");

% wayPoints = [start(1,4) start(3,4) start(2,4);0.04 0.5 0.5;0 0.35 0.25];
wayPoints = [start(1,4) start(3,4) start(2,4);0.13 0.4 0.55;0 0.35 0.3]; 

wayPointVels = [0 0 0;0 0.1 0;0 0 0];
numTotalPoints = size(wayPoints,1)*100;
waypointTime = 5;
wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
trajTimes = linspace(0,wpTimes(end),numTotalPoints);
trajectory = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
                     'VelocityBoundaryCondition',wayPointVels');
hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'b','LineWidth',1);

ik = robotics.InverseKinematics('RigidBodyTree',edo);
weights = [0.01 0.01 0.01 1 1 1];
initialguess = edo.homeConfiguration;

maxRate = pi/2;

% Define quaternions at start and end
quatStart = tform2quat(start);
quatEnd = [quatStart(1,1) quatStart(1,2) -quatStart(1,3) quatStart(1,4)]; 

configSoln(1,:) = initialguess;

% check collision
collisionPoses = [];
collisionCount = 0;

for idx = 2:size(trajectory,2)
    % Use slerp to interpolate between quatStart and quatEnd
    t = (idx-1)/(size(trajectory,2)-1); % interpolation parameter, between 0 and 1
    quat = quatinterp(quatStart, quatEnd, t, 'slerp');

    % Convert quaternion to rotation matrix
    tform = quat2tform(quat);

    % Set the position part of the transform
    tform(1:3, 4) = trajectory(:,idx)';

    newConfigSoln = ik('link_6', tform, weights, initialguess);

    diffConfig = newConfigSoln - configSoln(idx-1,:);

    for j = 1:length(newConfigSoln)
        if abs(diffConfig(j)) > maxRate
            newConfigSoln(j) = configSoln(idx-1,j) + sign(diffConfig(j))*maxRate;
        end
    end

    % save new configsoln
    configSoln(idx,:) = newConfigSoln;

    % Check collisions
    if checkCollision(edo, newConfigSoln, 'SkippedSelfCollisions','parent')
        collisionPoses = [collisionPoses; newConfigSoln];
        collisionCount = collisionCount + 1;
    end

    initialguess = configSoln(idx,:);
end

% v = VideoWriter('draftnew.avi');
% framerate = 10;
% v.FrameRate = framerate;
% open(v);

set(gcf, 'Position', [200, 200, 840, 630]);
xs = [0.25 0.25 -0.25 -0.25];  
ys = [0.25 0.5 0.5 0.25];  
zs = [0.25 0.25 0.25 0.25];  
plot3(xs, ys, zs, 'r'); 
fill3(xs, ys, zs, 'r', 'FaceAlpha', 0.5); 

for idx = 1:size(trajectory,2)
    show(edo,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
    pause(0.1)
    % frame = getframe(gcf);
    % writeVideo(v,frame);
end

hold off
% close(v);

jointconfigsoln = configSoln;
save("jointconfigsoln.mat","jointconfigsoln");

%% regenerate before surgery in joint space to get the proper movement

clear;
clc;
edo = importrobot('edo_sim.urdf');
edo.DataFormat = 'column';
show(edo);

load("jointconfigsoln.mat", "jointconfigsoln");
startConfig = [0; 0; 0; 0; 0; 0];
goalConfig = jointconfigsoln(300, :)';
q = trapveltraj([startConfig, goalConfig], 100, 'EndTime', 3);

hold on

v = VideoWriter('jointspace.avi');
framerate = 10;
v.FrameRate = framerate;
open(v);

set(gcf, 'Position', [200, 200, 840, 630]);
xs = [0.25 0.25 -0.25 -0.25];  
ys = [0.25 0.5 0.5 0.25];  
zs = [0.25 0.25 0.25 0.25];  
plot3(xs, ys, zs, 'r'); 
fill3(xs, ys, zs, 'r', 'FaceAlpha', 0.5); 

for i = 1:size(q, 2)
    show(edo, q(:, i), 'PreservePlot', false, 'Frames', 'off');
    pause(0.1);
    frame = getframe(gcf);
    writeVideo(v,frame);
end

hold off
close(v);

%% surgery period start up section code
% run this section before running surgery simulink model

clc;
clear;

edo = importrobot('edo_sim.urdf');
load("jointconfigsoln.mat", "jointconfigsoln");
start = getTransform(edo,edo.homeConfiguration,"base_link","link_6");
quatStart = tform2quat(start);
quatEnd = [quatStart(1,1) quatStart(1,2) -quatStart(1,3) quatStart(1,4)];
qend = quat2dcm(quatEnd);
% smimport('edo_sim.urdf');



