% Mobile Robot Mapping lecture 02: SVD example
% load point cloud data and display

clear all; close all;

load dat

n = size(dat,1);
dim = size(dat,2);

%% plot
figure(1)
plot3(dat(:,1), dat(:,2), dat(:,3),'.')
hold on;
axis equal

%% select a building
tall_idx = find(dat(:,3)>35);
dist_idx = find(dat(:,1)>450);
idx = intersect(tall_idx, dist_idx);

% we got building points
bldgpts = dat(idx,:);
plot3(bldgpts(:,1), bldgpts(:,2), bldgpts(:,3),'ro');

%% fit a plane
bldgpts_center = median(bldgpts);
center_error = bldgpts - bldgpts_center;
[U,S,V]=svd(center_error);
normal_vector = V(:,3);   
plane_size = 20;

plot_plane(normal_vector, bldgpts_center, plane_size);
