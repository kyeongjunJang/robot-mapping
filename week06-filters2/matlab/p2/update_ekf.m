function update_ekf (idx, dx, dy, sx2, sy2)

global pose cov

% landmark x and y value
landmark_idx = [4+2*(idx-1),5+2*(idx-1)];
Lx = pose(4+2*(idx-1));
Ly = pose(5+2*(idx-1));

% current robot (x,y,theta) estimate
x_current = pose(1:3);
x = x_current(1);
y = x_current(2);
t = x_current(3);

% measurement
z = [dx, dy]';
R = diag([sx2,sy2]);

delta_trans = sqrt(z(1)^2+z(2)^2);
delta_rot1 = atan2(z(2),z(1));

if (isnan(Lx) && isnan(Ly))
    % initialize landmark
    Lx_init = x + delta_trans*cos(t + delta_rot1);
    Ly_init = y + delta_trans*sin(t + delta_rot1);
    pose(landmark_idx) = [Lx_init, Ly_init]';
    cov(landmark_idx,landmark_idx) = R;

else
    % update with landmark
    delta = [Lx-x Ly-y]';
    z_predict = [cos(-t) -sin(-t);
                 sin(-t) cos(-t)]*delta;
    
    % measurement jacobian
    low_H = [-cos(t), -sin(t),   cos(t)*(conj(Ly) - conj(y)) - sin(t)*(conj(Lx) - conj(x)),  cos(t), sin(t);
              sin(t), -cos(t), - cos(t)*(conj(Lx) - conj(x)) - sin(t)*(conj(Ly) - conj(y)), -sin(t), cos(t)];
    F_matrix = [diag([1,1,1]) zeros(3,20);
                zeros(2,3) zeros(2,2*idx-2) diag([1,1]) zeros(2, 20-2*idx)];
    H = low_H * F_matrix;

    % Kalman update equations
    K = cov * H'*inv(H*cov*H' + R);
    pose = pose + K*(z - z_predict);
    cov = (eye(23) - K*H)*cov;
end

