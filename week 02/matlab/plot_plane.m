function plot_plane (n,pt,s)
% plot_plane draws a plane with normal vector
%  n:  normal vector of the plane (3x1)
%  pt: a point on the plane (3x1)
%  s:  size of the plane to plot
%
%  plane equation is given ax + by + cz + d = 0
%  and thus n = (a,b,c)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2015.09.01      ak          created and written
%    2020.09.01      yg          Modified


a = n(1); b = n(2); c = n(3);
x = pt(1); y = pt(2); z = pt(3);
d = -a*x -b*y -c*z;

x_range = pt(1)-s:1:pt(1)+s;
y_range = pt(2)-s:1:pt(2)+s;
[X,Y] = meshgrid(x_range,y_range);
Z=-(a * X + b * Y + d)/c;
surf(X,Y,Z)
shading flat
xlabel('x'); ylabel('y'); zlabel('z')
