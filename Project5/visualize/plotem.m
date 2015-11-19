function plotem
% visualization for project 5

% param 1x5 array
%          - param = [a, b, w, h, theta]
%          - (a,b): the center of the rectangle
%          - (w,h): width and height of the rectangle > 0
%          - theta: the rotation angle of the rectangle
% style string
%          - plot style string

close all

A = dlmread('../path.txt');

% A is now:
% [ x y theta rotated ];
% we ignore theta when printing
% we change colors if we change direction

hold on 

wh = 1;

yo  =  0

%axis([-10 10 -10 10]);


%plotObstacles(0) %empty
%plotObstacles(1) %street
%plotObstacles(2) %block 
%plotObstacles(4); %small obstacles

plotObstacles(yo);

plotPath(A);
plotEnds(A);

return

function plotPath(A)

%% draw all points in blue
%for i = 1:length(A)
%%  if A(i,4) == 0
%%    style = 'b';
%%  else
%%    style = 'c';
%%  end  
%  plot(A(i,1),A(i,2),'b');
%end

plot(A(:,1),A(:,2),'b.');

return

function plotEnds(A)

% redraw start in green
plot(A(1,1),A(1,1),'g');

% redraw goal in red
plot(A(length(A),1),A(length(A),2),'r');

return
