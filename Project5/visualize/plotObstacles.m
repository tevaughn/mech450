function plotObstacles(chooser)

% street
if (chooser == 1)  
  DrawRectangleCorner(-10,-10,20,4)
  DrawRectangleCorner(-10,-4,10,8)
  DrawRectangleCorner(2,-4,8,8)
end

% block
if (chooser == 2)
  DrawRectangleCorner(-7,-2,14,4)  
end

% small
if (chooser == 3)
  DrawRectangleCorner(-8,-4,2,3)
  DrawRectangleCorner(2,-2,3,3)
  DrawRectangleCorner(7,6,3,2)
end

% convert bottom left to center
function DrawRectangleCorner(x,y,w,h)
DrawRectangle([x+w/2,y+h/2,w,h,0],'b')
  