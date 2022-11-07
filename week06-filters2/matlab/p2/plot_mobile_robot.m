function plot_mobile_robot (X, scale, dof)
% plot_mobile (X, dof)
%
% X:    mu vector 
% dof: '2d' or '3d' 
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------

dim = 0;
if strcmp(dof, '2d')
    dim = 3;
else
    dim = 6;
    display('not implemented yet');
    return;
end

% generate 3xn or 6xn matrix
X_mat = reshape(X,dim,[]);

% number of poses
n = size(X_mat,2);
hold on;

for ii=1:n
    X_i = X_mat(:,ii);
    plot_triangle (X_i, scale);
end

x_min = min(X_mat(1,:)); x_max = max(X_mat(1,:));
y_min = min(X_mat(2,:)); y_max = max(X_mat(2,:));

% adjust range
axis([x_min-1,x_max+1,y_min-1,y_max+1]);
grid on;
hold off;
axis equal;