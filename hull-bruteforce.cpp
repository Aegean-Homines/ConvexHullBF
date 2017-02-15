/*!
* \file hull-bruteforce.cpp
* \author Egemen Koku
* \date 14 Feb 2017
* \brief Implementation of @b hull-bruteforce.h
*
* \copyright Digipen Institute of Technology
* \mainpage Hull Bruteforce Implementation
*
*/

#include "hull-bruteforce.h"
#include <limits>
#include <iostream>

/**
* @brief == operator for the Point class
* @param arg2 Point to be checked with
* @returns true if this and arg2 is equal, false otherwise
*/
bool Point::operator==( Point const& arg2 ) const {
    return ( (x==arg2.x) && (y==arg2.y) );
}

/**
* @brief << operator overloading for printing the point
* @param os the output stream to be used
* @param p the point to be printed
* @returns output stream
*/
std::ostream& operator<< (std::ostream& os, Point const& p) {
	os << "(" << p.x << " , " << p.y << ") ";
	return os;
}

/**
* @brief >> operator overloading for filling the point fields
* @param os the input stream to be used
* @param p the point to be written on
* @returns input stream
*/
std::istream& operator>> (std::istream& os, Point & p) {
	os >> p.x >> p.y;
	return os;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//return value is (on left, on right)
/**
* @brief helper function for finding where a point lies with respect to a line
* @param p1x x coordinate of the first point
* @param p2x x coordinate of the second point
* @param qx x coordinate of the point to be checked
* @param p1y y coordinate of the first point
* @param p2y y coordinate of the second point
* @param qy y coordinate of the point to be checked
* @returns pair of isOnLeft - isOnRight
*/
std::pair<bool,bool> get_location(
		float const& p1x, //check which side of the line (p1x,p1y)-->(p2x,p2y) 
		float const& p1y, //point (qx,qy) is on
		float const& p2x,
		float const& p2y,
		float const& qx,
		float const& qy
		) 
{
	Point dir   = {p2x-p1x,p2y-p1y};
	Point norm  = {dir.y, -dir.x};
	Point point = {qx-p1x,qy-p1y};
	//scalar product is positive if on right side
	float scal_prod = norm.x*point.x + norm.y*point.y;
	return std::make_pair( (scal_prod<0), (scal_prod>0));
}

// Well apparently the function above also does this but in a cooler way
// If this gives me bad performance, just switch to the one above
/**
* @brief helper function for checking whether all points lie on one side or not
* @param point1Index index of the first point
* @param point2Index index of the second point
* @param points vector of points
* @returns true if all the points are on one side, false otherwise
*/
bool checkPoints(unsigned int point1Index, unsigned int point2Index, std::vector< Point > const& points) {
	bool isOnNegativeSide = false;
	bool isOnPositiveSide = false;
	Point const & point1 = points[point1Index];
	Point const & point2 = points[point2Index];
	float xDiff = point2.x - point1.x;
	float yDiff = point2.y - point1.y;

	for (unsigned int i = 0; i < points.size(); ++i) {
		if (i == point1Index || i == point2Index)
			continue;

		Point const & pointToCheck = points[i];
		float lineEquationResult = (pointToCheck.y - point1.y) * xDiff - (pointToCheck.x - point1.x) * yDiff;
		if (lineEquationResult > 0)
			isOnPositiveSide = true;
		else if (lineEquationResult < 0)
			isOnNegativeSide = true;

		if (isOnNegativeSide && isOnPositiveSide)
			return false;
			
	}

	return true;
}

/**
* @brief First brute force hull function
* @param points all points on the plane
* @returns a set of indices of points that form convex hull
*/
std::set<int> hullBruteForce ( std::vector< Point > const& points ) {
	int num_points = points.size();
	if ( num_points < 3 ) throw "bad number of points";

	std::set<int> hull_indices;
	for (unsigned int i = 0; i < points.size(); ++i) { // For each point in the points
		for (unsigned int j = i + 1; j < points.size(); ++j) { // Try all the other points in the points
			if(checkPoints(i, j, points)) {
				hull_indices.insert(i);
				hull_indices.insert(j);
			}
		}
	}
		
	return hull_indices;
}

/**
* @brief Recursive function for the second convex hull bruteforce function
* @param hullIndices indices of the hull for the current iteration
* @param points all points on the plane
*/
void findHullVertexRec(std::vector<int> & hullIndices, std::vector< Point > const& points) {
	int lastVertex = hullIndices.back();

	bool sameSide = true;
	float lastVertexX = points[lastVertex].x;
	float lastVertexY = points[lastVertex].y;
	float possibleVertexX, possibleVertexY;

	// have to do this to get rid of all the sign-compare warnings
	int pointsSize = points.size();

	for (int i = pointsSize-1; i >= 0; --i) {
		if (i == lastVertex)
			continue;
		possibleVertexX = points[i].x;
		possibleVertexY = points[i].y;
		for (int k = 0; k < pointsSize && sameSide; ++k) {
			if (k == i || k == lastVertex)
				continue;

			sameSide = sameSide && (!get_location(lastVertexX, lastVertexY, possibleVertexX, possibleVertexY, points[k].x, points[k].y).second);
		}

		if(sameSide) {
			hullIndices.push_back(i);
			break;
		}

		sameSide = true;
	}

	// base condition
	if (hullIndices.back() == hullIndices[0]) {
		hullIndices.pop_back();
		return;
	}

	return findHullVertexRec(hullIndices, points);
}

/**
* @brief Second brute force hull function
* @param points all points on the plane
* @returns a vector of indices of points that form convex hull
*/
std::vector<int> hullBruteForce2 ( std::vector< Point > const& points ) {
	int num_points = points.size();
	if ( num_points < 3 ) throw "bad number of points";

	int smallestPointIndex;
	int xMin = std::numeric_limits<int>::max();
	std::vector<int> hull_indices;
	float currentX;
	for (unsigned int i = 0; i < points.size(); ++i) {
		currentX = points[i].x;

		if(currentX < xMin ) {
			smallestPointIndex = i;
			xMin = currentX;
		}
	}

	hull_indices.push_back(smallestPointIndex);

	findHullVertexRec(hull_indices, points);


	return hull_indices;
}
