#include "hull-bruteforce.h"
#include <algorithm>
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

//returns a set of indices of points that form convex hull
std::set<int> hullBruteForce ( std::vector< Point > const& points ) {
	int num_points = points.size();
	//std::cout << "number of points " << num_points << std::endl;
	if ( num_points < 3 ) throw "bad number of points";

	std::set<int> hull_indices;

		
	return hull_indices;
}

std::vector<int> hullBruteForce2 ( std::vector< Point > const& points ) {
	int num_points = points.size();
	if ( num_points < 3 ) throw "bad number of points";

	std::vector<int> hull_indices;


	return hull_indices;
}
