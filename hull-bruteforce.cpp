#include "hull-bruteforce.h"
#include <limits>
#include <iostream>

bool Point::operator==( Point const& arg2 ) const {
    return ( (x==arg2.x) && (y==arg2.y) );
}

std::ostream& operator<< (std::ostream& os, Point const& p) {
	os << "(" << p.x << " , " << p.y << ") ";
	return os;
}

std::istream& operator>> (std::istream& os, Point & p) {
	os >> p.x >> p.y;
	return os;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//return value is (on left, on right)
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

//returns a set of indices of points that form convex hull
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
