% ECE6707 wheel odometry using gtsam

clear; close all;

% prepare gtsam
addpath(genpath('./gtsam_toolbox'));
import gtsam.*
graph = NonlinearFactorGraph;
initialEstimate = Values;

% system config
load('encoder.mat');
enc_cnt_L = enc_cnt(:,3);
enc_cnt_R = enc_cnt(:,4);

VEHICLE_TREAD = 1.65;
WHEEL_DIAMETER = 0.640;           
ENCODER_RESOLUTION = 2048;
DTOR = pi/180;

%% Add prior = node 1
% p0 = [0, 0, 0]; S0 = diag([0.1^2, 0.1^2, (0.02*DTOR)^2]);
idx = 1;        % node index
initialEstimate.insert(1, Pose2(0.0, 0.0, 0.0));
priorMean = TODO
priorNoise = TODO
graph.add ... TODO
idx = idx+1;

figure(1); axis equal;
%% go for all encoder count
for k = 2:size(enc_cnt,1)
    % calculate relative motion from wheel odometry
    dL = ((enc_cnt_L(k)-enc_cnt_L(k-1)) / ENCODER_RESOLUTION * WHEEL_DIAMETER * pi);
    dR = ((enc_cnt_R(k)-enc_cnt_R(k-1)) / ENCODER_RESOLUTION * WHEEL_DIAMETER * pi);
    dist = (dL+dR)/2;
    dth = (dR - dL) / VEHICLE_TREAD;
    dx = dist * cos (dth);
    dy = dist * sin (dth); 
    
    % Fixed covariance for odometry Q = diag([0.15^2, 0.05^2, (1*DTOR)^2]);
    % add this incrementally to the graph
    initialEstimate.insert(idx, Pose2(0.0, 0.0, 0.0));
    u = TODO
    Q = TODO
    graph.add ... TODO
    idx=idx+1;

    % optimize and plot        
    optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
    result = optimizer.optimizeSafely();
    hold on; plot2DTrajectory(result, 'b-*');
    
    pause(0.1);
end