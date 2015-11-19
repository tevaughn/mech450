function plotObstacles(chooser)

% street
if (chooser == 1)  
  DrawRectangleCorner(-10,-10,20,4);
  DrawRectangleCorner(-10,-4,10,8);
  DrawRectangleCorner(2,-4,8,8);

% block
elseif (chooser == 2)
  DrawRectangleCorner(-7,-2,14,4);

% current small
elseif (chooser == 3)
  DrawRectangleCorner(-8,-4,2,3);
  DrawRectangleCorner(2,-2,3,3);
  DrawRectangleCorner(7,6,3,2);

% new small
elseif (chooser == 4)

  DrawRectangleCorner(-6,-4,1,1);
  DrawRectangleCorner(-1.5,-5,3,3);
  DrawRectangleCorner(-5,2.5,4.5,.5);

  DrawRectangleCorner(-3.25,-1,6,.5);
  DrawRectangleCorner(3,6,2,3);
  DrawRectangleCorner(-3,-4,1,2);

  DrawRectangleCorner(2,-6,1,2);
  DrawRectangleCorner(-1.25,-8.5,1,2);
  DrawRectangleCorner(-4.75,4,1.5,5);

  DrawRectangleCorner(-2.5,-8,1,2);
  DrawRectangleCorner(2.5,1,2,2);
  DrawRectangleCorner(-2,0,2,.5);

  DrawRectangleCorner(-4.25,-2,.5,2);
  DrawRectangleCorner(-1,4,3,4);

  
endif

return

% convert bottom left to center
function DrawRectangleCorner(x,y,w,h)
DrawRectangle([x+w/2,y+h/2,w,h,0],'b');
return
