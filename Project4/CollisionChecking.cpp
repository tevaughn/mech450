///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Taylor Vaughn, Laura Bierwirth, Nick Merritt
// Date: 17 Sept 2015
///////////////////////////////////////

#include "CollisionChecking.h"
#include <cmath>


// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector <Rectangle> &obstacles) {
    for (Rectangle rect : obstacles)
        if (between(rect.x, rect.width + rect.x, x) && between(rect.y, rect.height + rect.y, y))
            return false;
    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles.  If the circle
// lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector <Rectangle> &obstacles) {

    for (Rectangle rect : obstacles) {

        // check if the center is too close to the obstacle
        // return false if the obstacle and the circle would intersect
        if (between(rect.x - radius, rect.x + rect.width + radius, x) && between(rect.y, rect.y + rect.height, y))
            return false;

        else if (between(rect.x, rect.x + rect.width, x) && between(rect.y - radius, rect.y + rect.height + radius, y))
            return false;

        // check if the rectangle's boundary points are too close to the center,
        // return false if the boundary points are inside the robot
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

// find the euclidean distance between two points
double dist(double x1, double y1, double x2, double y2) {
    return sqrt(pow((y2 - y1), 2) + pow((x2 - x1), 2));
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles.
// If the square lies outside of all obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector <Rectangle> &obstacles) {

    double halfLength = sideLength / 2;
    double upperX, upperY, lowerX, lowerY;

    double blX, blY;
    double tlX, tlY;
    double trX, trY;
    double brX, brY;

    upperX = halfLength;
    upperY = halfLength;
    lowerX = -halfLength;
    lowerY = -halfLength;

    //rotate the square, calculate bottom left X, bottom left Y, top left X, etc...
    blX = rotatedX(lowerX, lowerY, theta, x, y);
    blY = rotatedY(lowerX, lowerY, theta, x, y);
    tlX = rotatedX(lowerX, upperY, theta, x, y);
    tlY = rotatedY(lowerX, upperY, theta, x, y);
    trX = rotatedX(upperX, upperY, theta, x, y);
    trY = rotatedY(upperX, upperY, theta, x, y);
    brX = rotatedX(upperX, lowerY, theta, x, y);
    brY = rotatedY(upperX, lowerY, theta, x, y);

    int rectCount = -1;

    for (Rectangle rect: obstacles) {
        rectCount++;

        // check if square's center is inside rectangle
        if (between(rect.x, rect.x + rect.width, x) && between(rect.y, rect.y + rect.height, y))
            return false;

        // check if left side of robot intersects the rectangle
        else if (checkLineToRect(blX, blY, tlX, tlY, rect))
            return false;

        // check top side
        else if (checkLineToRect(tlX, tlY, trX, trY, rect))
            return false;

        // check bottom side
        else if (checkLineToRect(blX, blY, brX, brY, rect))
            return false;

        // check right side
        else if (checkLineToRect(brX, brY, trX, trY, rect))
            return false;

    }

    return true;
}

// rotate and translate the x-coordinate of a point
double rotatedX(double qx, double qy, double theta, double x, double y) {
    return (qx * cos(theta) - qy * sin(theta) + x);
}

// rotate and translate the y-coordinate of a point
double rotatedY(double qx, double qy, double theta, double x, double y) {
    return (qx * sin(theta) + qy * cos(theta) + y);
}

// returns true if the given line intersects any side of the given rectangle
bool checkLineToRect(double point1x, double point1y, double point2x,
                     double point2y, Rectangle rect) {

    if (doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x, rect.y, rect.x, rect.y + rect.height))
        return true;

    else if (doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x, rect.y, rect.x + rect.width, rect.y))
        return true;

    else if (doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x, rect.y + rect.height, rect.x + rect.width,
                                rect.y + rect.height))
        return true;

    else if (doLineSegmentsIntersect(point1x, point1y, point2x, point2y, rect.x + rect.width, rect.y, rect.x + rect.width,
                                rect.y + rect.height))
        return true;

    return false;

}

// check if target is between high and low.
bool between(double low, double high, double target) {
    if (((target >= low) && (target <= high)) || ((target <= low) && (target >= high)))
        return true;
    return false;
}

// Returns true if they intersect
bool doLineSegmentsIntersect(double robotPoint1x, double robotPoint1y,
                             double robotPoint2x, double robotPoint2y,
                             double obstaclePoint1x, double obstaclePoint1y,
                             double obstaclePoint2x, double obstaclePoint2y) {

    // calculate slopes: y2-y1/x2-x1
    double mRobot = (robotPoint2y - robotPoint1y) / (robotPoint2x - robotPoint1x);
    double mObstacle = (obstaclePoint2y - obstaclePoint1y) / (obstaclePoint2x - obstaclePoint1x);

    // calculate y-intercepts: b = -mx + y
    double bRobot = (mRobot * -robotPoint1x) + robotPoint1y;
    double bObstacle = (mObstacle * -obstaclePoint1x) + obstaclePoint1y;

    // if both line segments have the same slope, we assume they don't intersect
    if (mRobot == mObstacle) {
        return false;
    }

    double x, y;

    // if obstacle is vertical, calculate intersection using y-intercept
    if (mObstacle == INFINITY) {

        x = obstaclePoint1x;
        y = mRobot * x + bRobot;

    } else {

        y = bObstacle;

        if (mRobot == INFINITY)
            x = robotPoint1x;
        else if (mRobot == 0)
            x = obstaclePoint1x;
        else
            x = (y - bRobot) / mRobot;

    }

    // if the x,y point of intersection exists between both line segments end points, then the lines intersect
    if (between(robotPoint1x, robotPoint2x, x) && between(robotPoint1y, robotPoint2y, y) &&
        between(obstaclePoint1x, obstaclePoint2x, x) && between(obstaclePoint1y, obstaclePoint2y, y))
        return true;

    return false;
}

// Add any custom debug / development code here.  This code will be executed instead of the
// statistics checker (Project2.cpp).  Any code submitted here MUST compile, but will not be graded.

// no-op
void debugMode(const std::vector <Robot> & /*robots*/, const std::vector <Rectangle> & /*obstacles*/,
               const std::vector<bool> & /*valid*/) {
}
