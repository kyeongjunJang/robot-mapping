% slam101 lec 08 Uncertainty Propagation
% Example in R. Smith 1990 paper

clear all; close all; clc;

DTOR = pi/180;
RTOD = 180/pi;

% robot starts at the origin = world frame
xr = [0 0 0]'; Sr = zeros(3,3);

% robot sees the object #1
x1 = [-1 1 0]'; R = diag([0.1^2, 0.2^2, (1*DTOR)^2]);
X = [xr; x1];
Cov = [Sr zeros(3,3);
       zeros(3,3) R];

% robot moves
u = [1 1 0]'; Q = diag([0.15^2, 0.05^2, (1*DTOR)^2]);
[yr, J] = head2tail_2d(xr,u);
Sy = J*[zeros(3,3) zeros(3,3);
     zeros(3,3) Q]*J';

X = [yr; x1];
Cov = [Sy zeros(3,3);
       zeros(3,3) R];

% robot sees #2
z2 = [-1 1 0]'; R = diag([0.1^2, 0.2^2, (1*DTOR)^2]);
[x2, J] = head2tail_2d(yr,z2);
S2 = J*Cov*J';
J1_oplus = J([1:3],[1:3]);

X = [yr; x1; x2];
Cov = [Sy zeros(3,3) Sy*J1_oplus';
       zeros(3,3) R zeros(3,3);
       J1_oplus*Sy, zeros(3,3), R+Q];

plot_pose_with_cov_ellipse (X, Cov)