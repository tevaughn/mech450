% visualization for project 5

% param 1x5 array
%          - param = [a, b, w, h, theta]
%          - (a,b): the center of the rectangle
%          - (w,h): width and height of the rectangle > 0
%          - theta: the rotation angle of the rectangle
% style string
%          - plot style string
close all

A = dlmread('path.txt');
% A is now:
% [ x y theta rotated ];
% we ignore theta when printing
% we change colors if we change direction

axis equal
hold on 

wh = 1;

%plotObstacles(1) %street
%plotObstacles(2) %block
%plotObstacles(3); %small obstacles

plotObstacles(4);

% draw all points in blue
for i = 1:length(A)
  if A(i,4) == 0
    style = 'b';
  else
    style = 'c';
  end  
  plot(A(i,1),A(i,2),style);
end

% redraw start in green
plot(A(1,1),A(1,1),'g');

% redraw goal in red
plot(A(i,1),A(i,2),'r');
