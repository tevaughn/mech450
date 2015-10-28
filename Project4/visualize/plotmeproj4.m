
function plotmeproj4

% car
clear all; close all

hold on

% % TODO obstacles

% obstacle 1, bottom
rectangle('Position',[-6,-10,12,4]);

% obstacle 2, bottom
rectangle('Position',[-6,-4,6,8]);

% obstacle 3, bottom
rectangle('Position',[2,-4,4,8]);

% obstacle 4, bottom
rectangle('Position',[-6,6,12,2]);
 
A = importdata('/Users/nicholasmerritt/programs/comp450/mech450/Project4/path.txt');

plotemcoach(A,-5,-5,5,5);




function plotemcoach(A, startx, starty, endx, endy)
r = .1;

for i = 1:length(A)
    gosqr(A(i,1),A(i,2),A(i,3), r);
    %plot(A(i,1),A(i,2))
end

rectangle('Position',[startx - r/2,starty - r/2,r,r], 'EdgeColor','g')
rectangle('Position',[endx - r/2,endy - r/2,r,r], 'EdgeColor','r')

axis equal

function gosqr(x,y,theta,r)

a = x;
b = y;
w = r;
h = r;
X = [-w/2 w/2 w/2 -w/2 -w/2];
Y = [h/2 h/2 -h/2 -h/2 h/2];
P = [X;Y];
ct = cos(theta);
st = sin(theta);
R = [ct -st;st ct];
P = R * P;
h=plot(P(1,:)+a,P(2,:)+b);
