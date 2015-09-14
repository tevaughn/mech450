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
    // IMPLEMENT ME!
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
    // IMPLEMENT ME!
	for (Rectangle rect : obstacles) {
		if ( doesVerticeInstersectCircle(x, y, radius, rect) || ((x + radius >= rect.x) && (x <= rect.width + rect.x + radius) && (y + radius >= rect.y) && (y <= rect.height + rect.y + radius)) ) {
			return false;
		}
	}

    return true;
}


// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles.
// If the square lies outside of all obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles)
{
    // IMPLEMENT ME!

    return false;
}

// Add any custom debug / development code here.  This code will be executed instead of the
// statistics checker (Project2.cpp).  Any code submitted here MUST compile, but will not be graded.
void debugMode(const std::vector<Robot>& /*robots*/, const std::vector<Rectangle>& /*obstacles*/, const std::vector<bool>& /*valid*/)
{
}
