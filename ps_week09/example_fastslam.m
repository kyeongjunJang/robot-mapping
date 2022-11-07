% ECE6707 Fast SLAM

clear all; close all; clc;

DTOR = pi/180;
RTOD = 180/pi;

n = 10;
weight = 1/n.*ones(n,1);

% robot starts at the origin = world frame
m0 = [0 0 0]'; S0=diag([0.1^2, 0.1^2, (0.01*DTOR)^2]);
pose0 = mvnrnd(m0,S0,n)';

figure(1); hold on;
plot (pose0(1,:), pose0(2,:), 'b.');

% robot moves 
Q = diag([0.1,0.1,1*DTOR].^2);
u = [1.9 4.1 -pi/4]';

pose1 = zeros(3,100);
for ii=1:n
    for jj=1:n
        w = mvnrnd([0,0 0]',Q,1)';
        pose1(:,ii+10*(jj-1)) = head2tail_2d(pose0(:,ii), u) + w;
    end
end

plot (pose1(1,:), pose1(2,:), 'b.');

x_p = mean(pose1');
mean_pose_x = x_p(1);
mean_pose_y = x_p(2);

%% (a) use observation #1 to compute the probability for each particle
z1 = 2; R1 = 1;
H = [1 0 0];
z_sample = H*pose1;
n = size(pose1, 2);
p = zeros(n,1);
for ii=1:n
    p(ii) = mvnpdf(z_sample(ii), z1, R1);
end

% update weight
weight = 1/n.*ones(n,1);
weight = weight.*p;

%% (b) observation all measurements #1,2 and 3
z1 = 2; R1 = 1;
z2 = 4; R2 = 1;
z3 = 5; R3 = 1;

d = x_p(1)^2 + x_p(2)^2;
H = [1 0 0;
     0 1 0;
     x_p(1)/sqrt(d) x_p(2)/sqrt(d) 0];
% H = TODO

% measurement
% z = TODO
% R = TODO
z = [z1; z2; z3];
R = diag([R1,R2,R3]);

z_sample = H*pose1;
n = size(pose1, 2);
p = zeros(n,1);
% p = TODO
for ii=1:n
    p(ii) = mvnpdf(z_sample(:, ii),z,R);
end

% update weight
weight = 1/n.*ones(n,1);
weight = weight.*p;

%% (c) what is the best so far?
[~, idx] = max(weight);
pose_best = pose1(:,idx);
plot (pose_best(1), pose_best(2), 'k*'); grid on; axis([-0.5,3,-0.5,5]);
