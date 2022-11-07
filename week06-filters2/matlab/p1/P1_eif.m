% PS week 06
% P1. EIF measurement update

clear all; close all;

DTOR = pi/180;
RTOD = 180/pi;

% robot starts at the origin = world frame
x0 = [0 0 0]'; S0=diag([0.1^2, 0.1^2, (0.01*DTOR)^2]);

%% robot moves 
Q = diag([0.1,0.1,1*DTOR].^2);
w = mvnrnd([0,0 0]',Q,1)';
u0 = [1.9 4.1 -pi/4]';

% make a prediction
[x1_p,J] = head2tail_2d(x0, u0);
x1_true = x1_p + w;
S1_p = J(:,1:3)*S0*J(:,1:3)'+Q;

X_estim = [x0; x1_p];
X_true = [x0; x1_true];

figure(1);
scale = 0.1;
plot_mobile_robot (X_estim, scale, '2d')
hold on
plot_ellipse (x0(1:2),S0(1:2,1:2),'r');
plot_ellipse (x1_p(1:2),S1_p(1:2,1:2),'r');
plot(x1_true(1),x1_true(2),'rx');
hold off;

%% measurement update
z1 = 2; R1 = 1;
z2 = 4; R2 = 1;
z3 = 5; R3 = 1;

z = [z1; z2; z3];
R = diag([R1,R2,R3]);

d = x1_p(1)^2 + x1_p(2)^2;
H = [1 0 0;
     0 1 0;
     x1_p(1)/sqrt(d) x1_p(2)/sqrt(d) 0];

L1_p = inv(S1_p); % Large Gamma
eta_p = L1_p * x1_p;

z_p = [x1_p(1); x1_p(2); sqrt(x1_p(1)^2 + x1_p(2)^2)];

L1 = L1_p + H'*inv(R)*H;
eta = eta_p + H'*inv(R)*(z - z_p + H*x1_p);

S1 = inv(L1);
x1 = linsolve(L1,eta);

figure(2);
X_estim = [x0; x1];
plot_mobile_robot (X_estim, scale, '2d')
hold on
plot_ellipse (x0(1:2),S0(1:2,1:2),'r');
plot_ellipse (x1_p(1:2),S1_p(1:2,1:2),'r');
plot(x1_true(1),x1_true(2),'rx');