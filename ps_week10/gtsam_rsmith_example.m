% ECE6707 Uncertainty Propagation using gtsam
% Example in R. Smith 1990 paper

clear all; close all; clc;
DTOR = pi/180; RTOD = 180/pi;

% prepare gtsam
addpath(genpath('./gtsam_toolbox'));
import gtsam.*
graph = NonlinearFactorGraph;
initialEstimate = Values;

%% (a) robot starts at the origin. Add a prior = node1
% xr = [0 0 0]'; Sr = zeros(3,3);
initialEstimate.insert(1, Pose2(0.0, 0.0, 0.0));
priorMean = Pose2(0.0, 0.0, 0.0);
priorNoise = noiseModel.Diagonal.Sigmas([0.1; 0.2; 1*DTOR]);
graph.add(PriorFactorPose2(1, priorMean, priorNoise)); % add directly to graph

%% (b) robot sees the object #1 = node2
% z1 = [-1 1 0]'; R = diag([0.1^2, 0.2^2, (1*DTOR)^2]);
initialEstimate.insert(2, Pose2(0.0, 0.0, 0.0));
z1 = Pose2(-1.0, 1.0, 0.0);
R = noiseModel.Diagonal.Sigmas([0.1; 0.2; 1*DTOR]); % measurementNoise
graph.add(BetweenFactorPose2(1, 2, z1, R));

%% (c) robot move to (1,1) = node3
% u = [1 1 0]'; Q = diag([0.15^2, 0.05^2, (1*DTOR)^2]);
initialEstimate.insert(3, Pose2(0.0, 0.0, 0.0));
u = Pose2(1.0, 1.0, 0.0);
Q = noiseModel.Diagonal.Sigmas([0.15; 0.05; 1*DTOR]); % odometryNosie
graph.add(BetweenFactorPose2(1, 3, u, Q));

%% (d) robot see the object #2 = node4
% z2 = [-1 1 0]'; R = diag([0.1^2, 0.2^2, (1*DTOR)^2]);
initialEstimate.insert(4, Pose2(0.0, 0.0, 0.0));
z2 = Pose2(-1.0, 1.0, 0.0);
graph.add(BetweenFactorPose2(3, 4, z2, R));

%% optimize and plot
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
plot2DTrajectory(result, 'b-*');
  