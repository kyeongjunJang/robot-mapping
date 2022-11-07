clc; clear; close all;

VEHICLE_TREAD = 1.65;
WHEEL_DIAMETER = 0.640;           
ENCODER_RESOLUTION = 2048;

load('encoder.mat');
enc_cnt_L = enc_cnt(:,3);
enc_cnt_R = enc_cnt(:,4);

pose_global = [0, 0, 0];
pose_var = [0, 0, 0]; % [var_dx, var_dy, var_dth]

interval = 10;
for i = 1:size(enc_cnt,1)-1
    dL = pi*WHEEL_DIAMETER/ENCODER_RESOLUTION*(enc_cnt_L(i+1) - enc_cnt_L(i));
    dR = pi*WHEEL_DIAMETER/ENCODER_RESOLUTION*(enc_cnt_R(i+1) - enc_cnt_R(i));
    
    dist = (dR+dL)/2;
    dth = atan((dR-dL)/VEHICLE_TREAD);

    dx = dist*cos(dth);
    dy = dist*sin(dth);

    pose_x = dx*cos(pose_global(end,3)) - dy*sin(pose_global(end,3)) + pose_global(end,1);
    pose_y = dx*sin(pose_global(end,3)) + dy*cos(pose_global(end,3)) + pose_global(end,2);
    pose_th = pose_global(end,3)+dth;
    
    pose_global = [pose_global; [pose_x, pose_y, pose_th]];
%     pose_var = [pose_var; [var_dx, var_dy, var_dth]];
end

plot(pose_global(:,1), pose_global(:,2)); hold on; % plot trajectory
plot(pose_global(1:20:end,1), pose_global(1:20:end,2), 'k.'); % plot sample position
grid on; axis equal; xlabel('x'); ylabel('y');

