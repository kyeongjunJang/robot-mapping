function plot_landmarks (X, dof)
% plot_landmarks (X, dof)
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
    dim = 2;
else
    dim = 3;
    display('not implemented yet');
    return;
end

% generate 3xn or 6xn matrix
X_mat = reshape(X,dim,[]);

% number of poses
n = size(X_mat,2);
valididx = find(~isnan(X_mat(1,:)));
n_valid = length(valididx);

for ii=1:n_valid
    idx = valididx(ii);
    X_i = X_mat(:,idx);
    plot(X_i(1), X_i(2), 'bx', 'LineWidth', 5);
end