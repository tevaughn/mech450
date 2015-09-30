#ifndef COLLISION_CHECKING_H_
#define COLLISION_CHECKING_H_

#include <vector>
#include <cmath>
#include <iostream>

struct Rectangle
{
    // Coordinate of the lower left corner of the rectangle
    double x, y;
    // The width (x-axis extent) of the rectangle
    double width;
    // The height (y-axis extent) of the rectangle
    double height;
};

// Definition of our robot.
struct Robot
{
    // Type = {c,s,p}.  Circle, square, or point robot
    char type;
    // The location of the robot in the environment
    double x, y;
    // The orientation of the square robot.  Undefined for point or circle robot
    double theta;
    // The length of a side of the square robot or the radius of the circle robot
    // Undefined for the point robot.
    double length;
};

// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles);

// Intersect a circle with center (x,y) and given radius with the set of rectangles.  If the circle
// lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle>& obstacles);

// euclidean distance between two points
double dist (double x1, double y1, double x2, double y2);

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles.
// If the square lies outside of all obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles);

// rotate and transform x and y coordinates of robots
double rotatedX(double qx, double qy, double theta, double x, double y);
double rotatedY(double qx, double qy, double theta, double x, double y);

// check if a line intersects a given rectangle
bool checkLineToRect(double point1x, double point1y, double point2x, double point2y, Rectangle rect);

// find if two line segments intersect, given their endpoints
bool doLineSegmentsIntersect(double A1x, double A1y, double A2x, double A2y, double B1x, double B1y, double B2x, double B2y);

// check if low <= target <= high or vice versa
bool between(double low, double high, double target);


// Custom debugging/development code.  Takes the list of robots, the list of obstacles,
// and whether or not each configuation should be valid or not.
void debugMode(const std::vector<Robot>& robots, const std::vector<Rectangle>& obstacles, const std::vector<bool>& valid);

#endif

