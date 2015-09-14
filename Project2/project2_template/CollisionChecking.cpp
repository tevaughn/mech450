///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors:
// Date:
//////////////////////////////////////

#include "CollisionChecking.h"


// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    // IMPLEMENT ME!
	for (Rectangle rect : obstacles) {
		if ( (x >= rect.x) && (x >= rect.width + rect.x) && (y >= rect.y) && (y >= rect.height + rect.y) ) {
			return false;
		}
	}

    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles.  If the circle
// lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle>& obstacles)
{
    // IMPLEMENT ME!

    return false;
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


	lowerX = x - halfLength;
	lowerY = y - halfLength;
	upperX = x + halfLength;
	upperY = y + halfLength;
	
	blX = rotatedX(lowerX, lowerY, theta, x, y);
	blY = rotatedY(lowerX, lowerY, theta, x, y);
	tlX = rotatedX(lowerX, upperY, theta, x, y);
	tlY = rotatedY(lowerX, upperY, theta, x, y);
	trX = rotatedX(upperX, upperY, theta, x, y);
	trY = rotatedY(upperX, upperY, theta, x, y);
	brX = rotatedX(upperX, lowerY, theta, x, y);
	brY = rotatedY(upperX, lowerY, theta, x, y);

	for (Rectangle rect: obstacles) {
		if (checkLineToRect(blX, blY, tlX, tlY, rect)) {
			std::cout << "returning false a\n";
			return false;
		}
		if (checkLineToRect(tlX, tlY, trX, trY, rect)) {
			std::cout << "returning false b\n";
			return false;
		}
		if (checkLineToRect(blX, blY, brX, brY, rect)) {
			std::cout << "returning false c\n";
			return false;
		}
		if (checkLineToRect(brX, brY, trX, trY, rect)) {
			std::cout << "returning false d\n";
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
	
	return (
		doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x, rect.y, rect.x, rect.y+rect.height) || 
		doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x, rect.y, rect.x+rect.width, rect.y) || 
		doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x, rect.y+rect.height, rect.x+rect.width, rect.y+rect.height) || 
		doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x+rect.width, rect.y, rect.x+rect.width, rect.y+rect.height));

}

bool doLineSegmentsIntersect(double robotPoint1x, double robotPoint1y, 
	double robotPoint2x, double robotPoint2y, double obstaclePoint1x, 
	double obstaclePoint1y, double obstaclePoint2x, double obstaclePoint2y) {

	// calculate slopes
	double mRobot = (robotPoint2y - robotPoint1y) / (robotPoint2x - robotPoint1x);
	double mObstacle = (obstaclePoint2y - obstaclePoint1y) / (obstaclePoint2x - obstaclePoint1x);

	// calculate y-intercepts
	double bRobot = (mRobot * -robotPoint1x) + robotPoint1y;
	double bObstacle = (mObstacle * -obstaclePoint1x) + obstaclePoint1y;

	// both segments have the same slope
	if (mRobot == mObstacle) {
		
		//std::cout << "equal slopes of " << mRobot << " and " << mObstacle << "\n";
		/*
		if (bRobot != bObstacle) {
			std::cout << "brobot parallel to bobstacle\n";
			return false;		
		}
		else if ((((robotPoint1x < obstaclePoint1x) && (obstaclePoint1x < robotPoint2x)) || ((robotPoint2x < obstaclePoint1x) && (obstaclePoint1x < robotPoint1x))) && (((robotPoint1y < obstaclePoint1y) && (obstaclePoint1y < robotPoint2y)) || ((robotPoint2y < obstaclePoint1y) && (obstaclePoint1y < robotPoint1y)))) {
			std::cout << "why is this happening a\n";
			return true;
		}
		else if ((((robotPoint1x < obstaclePoint2x) && (obstaclePoint2x < robotPoint2x)) || ((robotPoint2x < obstaclePoint2x) && (obstaclePoint2x < robotPoint1x))) && (((robotPoint1y < obstaclePoint2y) && (obstaclePoint2y < robotPoint2y)) || ((robotPoint2y < obstaclePoint2y) && (obstaclePoint2y < robotPoint1y)))) {
			std::cout << "why is this happening b\n";
			return true;
		}

		if (mObstacle == INFINITY) {
			((obstaclePoint1y > robotPoint1y && obstaclePoint1y > robotPoint2y) || (obstaclePoint2y > robotPoint1y && obstaclePoint2y > robotPoint2y)
		}
		else {
			std::cout << "brobot and bobstacle are the same but don't intersect\n";
			return false;
		}*/
		return false;
	}

	double x, y;

	if (mObstacle == INFINITY) {
		x = obstaclePoint1x;
		y = mRobot*x + bRobot;	
	} else {
		y = obstaclePoint1y;
		x = (y - bRobot)/mRobot;	
	}

	std::cout << "mobstacle = " << mObstacle << "  mrobot = " << mRobot << "\n";
	std::cout << "x-intersection = " << x << " y-interception = " << y << "\n";

	if (((((robotPoint1x <= x) && (x <= robotPoint2x)) 
		|| ((robotPoint2x <= x) && (x <= robotPoint1x))) 
		&& (((robotPoint1y <= y) && (y <= robotPoint2y)) 
		|| ((robotPoint2y <= y) && (y <= robotPoint1y)))) 
		&& ((((obstaclePoint1x <= x) && (x <= obstaclePoint2x)) 
		|| ((obstaclePoint2x <= x) && (x <= obstaclePoint1x))) 
		&& (((obstaclePoint1y <= y) && (y <= obstaclePoint2y)) 
		|| ((obstaclePoint2y <= y) && (y <= obstaclePoint1y))))) {
		return true;
	}
	return false;
} 

// Add any custom debug / development code here.  This code will be executed instead of the
// statistics checker (Project2.cpp).  Any code submitted here MUST compile, but will not be graded.
void debugMode(const std::vector<Robot>& /*robots*/, const std::vector<Rectangle>& /*obstacles*/, const std::vector<bool>& /*valid*/)
{
}
