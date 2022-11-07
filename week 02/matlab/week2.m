clear all; close all;

A = [15, -5; -5, 15]
% A = [-0.707106781186548,-0.707106781186548;-0.707106781186548,0.707106781186548]
% A = [0.8, 0.3;0.2,0.7]
% A = [2,1;1,2]
[U, S] = eig(A)

eigshow(A)

B = [1, 0; 0, 1]*[10, 0; 0, 5]*[1, 0; 0, 1];

eigshow(B)

theta = pi/2;
C = [cos(theta), -sin(theta); sin(theta), cos(theta)]*...
[10, 0; 0,5]*...
[cos(theta), sin(theta); -sin(theta), cos(theta)];

eigshow(C)
