///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors:
// Date:
//////////////////////////////////////

#include "CollisionChecking.h"
#include <cmath>


// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
	for (Rectangle rect : obstacles) {
		if ( (x >= rect.x) && (x <= rect.width + rect.x) && (y >= rect.y) && (y <= rect.height + rect.y) ) {
			return false;
		}
	}
    return true;

}

bool doesVerticeInstersectCircle(double x, double y, double radius, Rectangle rect) {
	double rad2 = pow(radius, 2);
	double xmin2 = pow(rect.x - x, 2);
	double xmax2 = pow(x - (rect.x + rect.width), 2);
	double ymin2 = pow(rect.y - y , 2);
	double ymax2 = pow(y - (rect.y + rect.height), 2);
	
	if (( (xmin2 + ymin2) <= rad2 ) ||
		( (xmax2 + ymin2) <= rad2 ) ||
		( (xmin2 + ymax2) <= rad2 ) ||
		( (xmax2 + ymax2) <= rad2 )) {
		return true; // not all vertices are > r from center of circle
	}

	return false;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles.  If the circle
// lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle>& obstacles)
{

    for (Rectangle rect : obstacles) {

        //line 1
        if ((rect.x - radius <= x) && (x <= rect.x + rect.width + radius) && (rect.y <= y) && (y <= rect.y + rect.height)) {
            return false;
        }

        //line 2
        else if ((rect.x <= x) && (x <= rect.x + rect.width) && (rect.y - radius <= y) && (y <= rect.y + rect.height + radius)) {
            return false;
        }

        //line 3
        else if (dist(rect.x, rect.y, x, y) <= radius)
            return false;
        else if (dist(rect.x + rect.width, rect.y, x, y) <= radius)
            return false;
        else if (dist(rect.x, rect.y + rect.height, x, y) <= radius)
            return false;
        else if (dist(rect.x + rect.width, rect.y + rect.height, x, y) <= radius)
            return false;
    }

    return true;
}

double dist (double x1, double y1, double x2, double y2) {
    return sqrt(pow((y2-y1), 2) + pow((x2-x1), 2));
}


// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles.
// If the square lies outside of all obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles)
{
    // IMPLEMENT ME!

	double halfLength = sideLength/2;
	double upperX, upperY, lowerX, lowerY;
	double blX, blY; 
	double tlX, tlY;
	double trX, trY;
	double brX, brY;


	lowerX = -halfLength;
	lowerY = -halfLength;
	upperX = halfLength;
	upperY = halfLength;


	blX = rotatedX(lowerX, lowerY, theta, x, y);
	blY = rotatedY(lowerX, lowerY, theta, x, y);
	tlX = rotatedX(lowerX, upperY, theta, x, y);
	tlY = rotatedY(lowerX, upperY, theta, x, y);
	trX = rotatedX(upperX, upperY, theta, x, y);
	trY = rotatedY(upperX, upperY, theta, x, y);
	brX = rotatedX(upperX, lowerY, theta, x, y);
	brY = rotatedY(upperX, lowerY, theta, x, y);

	std::cout << "(" << x << "," << y << ") -> theta:" << theta << " bl:(" << blX << "," << blY << "), tl:(" << tlX << "," << tlY << ") \n";

	int rectCount = -1;

	for (Rectangle rect: obstacles) {
		rectCount++;

		if (checkLineToRect(blX, blY, tlX, tlY, rect)) {
			std::cout << "returning false, intersects robot's left " << rectCount << "\n";
			return false;
		}
		if (checkLineToRect(tlX, tlY, trX, trY, rect)) {
			std::cout << "returning false, intersects robot's top " << rectCount << "\n";
			return false;
		}
		if (checkLineToRect(blX, blY, brX, brY, rect)) {
			std::cout << "returning false, intersects robot's bottom " << rectCount << "\n";
			return false;
		}
		if (checkLineToRect(brX, brY, trX, trY, rect)) {
			std::cout << "returning false, intersects robot's right " << rectCount << "\n";
			return false;
		}
	}

	std::cout << "returning true\n";
    return true;
}

double rotatedX(double qx, double qy, double theta, double x, double y) {
	return (qx*cos(theta) - qy*sin(theta) + x);
}

double rotatedY(double qx, double qy, double theta, double x, double y) {
	return (qx*sin(theta) + qy*cos(theta) + y);
}

// returns true if any intersects
bool checkLineToRect(double point1x, double point1y, double point2x, 
		double point2y, Rectangle rect) {
	
	if (doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x, rect.y, rect.x, rect.y+rect.height) ) {
		std::cout << "intersects left side\n";		
		return true;
	}
	if (doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x, rect.y, rect.x+rect.width, rect.y)) {
		std::cout << "intersects bottom side\n";	
		return true;
	}
	if (doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x, rect.y+rect.height, rect.x+rect.width, rect.y+rect.height)) {
		std::cout << "intersects top side\n";	
		return true;	
	}
	if (doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x+rect.width, rect.y, rect.x+rect.width, rect.y+rect.height)) {
		std::cout << "intersects right side\n";	
		return true;
	}
	
	return false;

}

bool between(double low, double high, double target) {
	
	if ( ((target >= low) && (target <= high)) || ((target <= low) && (target >= high))  ) {
		return true;	
	}
	return false;
}

// Returns true if they intersect
bool doLineSegmentsIntersect(double robotPoint1x, double robotPoint1y, 
	double robotPoint2x, double robotPoint2y, double obstaclePoint1x, 
	double obstaclePoint1y, double obstaclePoint2x, double obstaclePoint2y) {

	// calculate slopes y2-y1/x2-x1
	double mRobot = (robotPoint2y - robotPoint1y) / (robotPoint2x - robotPoint1x);
	double mObstacle = (obstaclePoint2y - obstaclePoint1y) / (obstaclePoint2x - obstaclePoint1x);

	// calculate y-intercepts b = -mx + y
	double bRobot = (mRobot * -robotPoint1x) + robotPoint1y;
	double bObstacle = (mObstacle * -obstaclePoint1x) + obstaclePoint1y;

	// if both line segments have the same slope, we assume they don't intersect
	if (mRobot == mObstacle) {
		return false;
	}

	double x, y;

	if (mObstacle == INFINITY) {
		x = obstaclePoint1x;
		y = mRobot*x + bRobot;
	} else {
		y = bObstacle;
		if (mRobot == INFINITY) {
			x = robotPoint1x;
		} else if (mRobot == 0 ){
			x = obstaclePoint1x;
		} else {
			x = (y - bRobot)/mRobot;
		}	
	}


	std::cout << "mobstacle = " << mObstacle << "  mrobot = " << mRobot << "\n";
	std::cout << "x-intersection = " << x << " y-interception = " << y << " robot line:(" << robotPoint1x << "," << robotPoint1y << "),(" << robotPoint2x << "," << robotPoint2y << ") obstacle line:(" << obstaclePoint1x << "," << obstaclePoint1y << "),(" << obstaclePoint2x << "," << obstaclePoint2y << ")\n";

	// if the x,y point of intersection exists between both line segments end points, then the lines intersect
	if ( between(robotPoint1x, robotPoint2x, x) && between(robotPoint1y, robotPoint2y, y) &&
			between(obstaclePoint1x, obstaclePoint2x, x) && between(obstaclePoint1y, obstaclePoint2y, y) ) {
		return true;
	}
	return false;
} 

// Add any custom debug / development code here.  This code will be executed instead of the
// statistics checker (Project2.cpp).  Any code submitted here MUST compile, but will not be graded.
void debugMode(const std::vector<Robot>& /*robots*/, const std::vector<Rectangle>& /*obstacles*/, const std::vector<bool>& /*valid*/)
{
}
