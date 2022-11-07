% slam101 lec 08 Uncertainty Propagation
% Example in R. Smith 1990 paper

clear all; close all;

DTOR = pi/180;
RTOD = 180/pi;

% robot starts at the origin = world frame
xr = [0 0 0]'; Sr = zeros(3,3);

% robot sees the object #1
x1 = [-1 1 0]'; R = diag([0.2^2, 0.2^2, (1*DTOR)^2]);
X = [xr; x1];
Cov = [Sr zeros(3,3);
       zeros(3,3) R];

% robot moves
u = [1 1 0]'; Q = diag([0.15^2, 0.05^2, (1*DTOR)^2]);
[yr, J] = %TODO hint: you can use head2tail_2d func
Sy = %TODO

X = %TODO
Cov = %TODO

% robot sees #2
z2 = [-1 1 0]'; R = diag([0.1^2, 0.2^2, (1*DTOR)^2]);
[x2, J] = %TODO
S2 = %TODO

X = %TODO
Cov = %TODO

plot_pose_with_cov_ellipse (X, Cov)