% PS 06
% P2. Landmark#10 SLAM example 

clear all; close all;

global pose cov % global variables

% state vector = [Xr L1 L2 ... L10]
n_landmark = 10;
pose = nan(3+2*n_landmark,1);
cov = zeros(3+2*n_landmark,3+2*n_landmark);

% intialize pose
pose(1:3) = [0,0,0]';

% trajectory
n_step = 100;
robot_traj = zeros(3,n_step);
traj_idx = 0;

% file open
fid = fopen ('landmark.txt');

tline = fgets(fid);
while ischar(tline) 
    if (tline(1) ~= '#') % skip comment line
        if (strcmp(tline(1:4),'ODOM'))
            [dx, dy, dt, sx2, sy2, st2] = strread(tline(5:end), '%f %f %f %f %f %f');
            propagate_ekf (dx, dy, dt, sx2, sy2, st2);
                        
            % update trajectory
            traj_idx = traj_idx + 1;
            robot_traj(:,traj_idx) = pose(1:3);
            
        elseif (strcmp(tline(1:8),'LANDMARK'))
            [idx, dx, dy, sx2, sy2] = strread(tline(10:end), '%d %f %f %f %f');
            update_ekf (idx, dx, dy, sx2, sy2);
        end
        
        % plot trajectory and landmarks
        figure(1);
        clf;
        scale = 0.5;
        plot_mobile_robot (reshape(robot_traj(:,1:traj_idx), 3*traj_idx, 1), scale, '2d')
        hold on
        plot_landmarks (pose(4:end),'2d');
        plot_ellipse (pose(1:2),cov(1:2,1:2),'r');
        hold off; grid on; axis equal; axis([-10,30,-10,40])
        drawnow;

    end
    tline = fgets(fid);
end

fclose (fid);