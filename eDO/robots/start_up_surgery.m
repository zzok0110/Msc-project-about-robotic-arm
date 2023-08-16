clc;
clear;

edo = importrobot('edo_sim.urdf');
load("jointconfigsoln.mat", "jointconfigsoln");
start = getTransform(edo,edo.homeConfiguration,"base_link","link_6");
quatStart = tform2quat(start);
quatEnd = [quatStart(1,1) quatStart(1,2) -quatStart(1,3) quatStart(1,4)];
qend = quat2dcm(quatEnd);
% smimport('edo_sim.urdf');
