function plot_triangle (ctr, scale)
%
% ctr:   center
% scale: size of trangle
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2015.11.05      ak          created and written
%    2019.11.05      yg          modified

xy = ctr(1:2); theta = ctr(3);

% some defining vertices
a = [0,sqrt(3)]'*scale;
b = [-1,0]'*scale;
c = [1,0]'*scale;

R = [cos(theta) -sin(theta);
     sin(theta) cos(theta)];
 
A = R*a+xy;
B = R*b+xy;
C = R*c+xy;

% collect coordinates
X = [A(1), B(1), C(1)]; % x values
Y = [A(2), B(2), C(2)]; % y values

% plot triangle
fill(X,Y, [0,0.4,0]); % green