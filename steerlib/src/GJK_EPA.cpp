/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"
using namespace Util;

SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Vector& return_penetration_vector, const std::vector<Vector>& _shapeA, const std::vector<Vector>& _shapeB)
{
	std::vector<Vector> simplex;
	bool is_colliding;
	is_colliding = GJK(_shapeA,_shapeB, simplex);

	if (is_colliding == true)
	{
		EPA(_shapeA, _shapeB, simplex, return_penetration_vector, return_penetration_depth);
		return true;
	}
	else 
	{
		return false;
	}
}

bool SteerLib::GJK_EPA::GJK(const std::vector<Vector>& _shapeA, const std::vector<Vector>& _shapeB, std::vector<Vector>& simplex)
{
	//direction d and farthest vertex v
	Vector d, v;

	//set initial d to be the vector from center of shapeB to center of shapeA
	//d = centerOfPolygon(_shapeB).operator - (centerOfPolygon(_shapeA));

	//set d to (0,-1)
	d.x = 0;
	d.z = -1;
	d.y = 0;

	//v is the farthest vertex in direction d
	v = support(_shapeA, _shapeB, d);
	//add v into simplex
	simplex.push_back(v);
	d = d.operator-();

	while (true)
	{
		//add another v into simplex
		v = support(_shapeA, _shapeB, d);
		simplex.push_back(v);
		//make sure the last point passed oringin
		if (simplex.back().operator*(d) <= 0)
		{
			return false;
		}
		else if (containOrigin(simplex, d))
		{
			return true;
		}
	}
}

// ------------------------------------------------		EPA		---------------------------------------------------------- //

void SteerLib::GJK_EPA::EPA(const std::vector<Vector>& _shapeA, const std::vector<Vector>& _shapeB, 
	std::vector<Vector>& simplex, Vector& perpendicular_vector, float& perpendicular_depth)
{
	// |p_distance - edge_distance|
	float epsilon = 0.001;
	while (true){
		// find the closest to origin edge in the current Simplex
		int edgeNumber = findClosestEdge(simplex);

		// obtain a new support point p in the direction of edge's normal 
		Vector n = edgeNormal(edgeNumber, simplex);
		Vector p = support(_shapeA, _shapeB, n);
		// compare p's distance to origin and edge's distance to origin
		// if |p_distance - edge_distance| < epsilon then we touch the most edge of Minkowski difference
		// which means that we find the solution.

		if (abs(p.operator*(n)) - abs(simplex[edgeNumber].operator*(n)) < epsilon){
			perpendicular_depth = abs(simplex[edgeNumber].operator*(n));
			perpendicular_vector = n;
			return;
		}
		else{
			// In this situation, we still don't meet the most edge of Minkowski difference.
			// What to do is keeping expanding the simplex, and add the new point between the edge
			// to maintain the convexity.
			insertSimplexPoint(simplex, p, edgeNumber);
		}
	}
}

int SteerLib::GJK_EPA::findClosestEdge(std::vector<Vector>& simplex)
{
	int closestEdgeIndex = 0;
	//the vertices A B C
	Vector a, b, c, d;
	//the edge AB, BC, AO, CO and normal
	Vector ab, cd, ao, co, abPerp, cdPerp;
	//the distance
	float d1, d2;
	std::vector<Vector>::size_type l = simplex.size();
	//get farthest vertex in direction d
	for (int i = 1; i < l; i++)
	{
		
		a = simplex[closestEdgeIndex];
		b = simplex[closestEdgeIndex + 1];
		c = simplex[i];
		
		if (i == l-1) {
			d = simplex[0];
		}
		else d = simplex[i + 1];

		//compute AB
		ab = b.operator-(a);
		//compute AO
		ao = a.operator-();
		//compute the normal
		abPerp = tripleProduct(ab, ao, ab);
		//the distance from AB to origin
		d1 = ao.operator*(abPerp) / abPerp.length();

		//compute CD
		cd = d.operator-(c);
		//compute CO
		co = c.operator-();
		//compute the normal
		cdPerp = tripleProduct(cd, co, cd);
		//the distance from CD to origin
		d2 = co.operator*(cdPerp) / cdPerp.length();
		if (abs(d2) < abs(d1))
		{
			closestEdgeIndex = (int)i;
		}
	}
	return closestEdgeIndex;
}

Vector SteerLib::GJK_EPA::edgeNormal(int edgeNumber, const std::vector<Vector>& simplex)
{
	// this must be normalized
	Vector n;
	Vector a, b, ab, ao;
	a = simplex[edgeNumber];
	if (edgeNumber == simplex.size()-1) {
		b = simplex[0];
	}
	else b = simplex[edgeNumber + 1];

	//compute AB
	ab = b.operator-(a);
	//compute AO
	ao = a.operator-();
	//compute the normal
	n = tripleProduct(ab, ao, ab);
	n = n.operator-().operator/(n.length());
	return n;
}

void SteerLib::GJK_EPA::insertSimplexPoint(std::vector<Vector>& simplex, Vector p, int edgeNumber)
{
	if (edgeNumber == simplex.size()-1) {
		simplex.push_back(p);
	}
	else simplex.insert(simplex.begin()+edgeNumber+1, p);
}


bool SteerLib::GJK_EPA::containOrigin(std::vector<Vector>& simplex, Vector& d)
{
	Vector a, b, c, ao, ab, ac, abPerp, acPerp;

	//last vertex in simplex
	a = simplex.back();
	//vector AO
	ao = a.operator-();

	//simplex is triangle
	if (simplex.size() == 3)
	{
		//get b and c
		b = simplex[0];
		c = simplex[1];
		//compute edges AB and AC
		ab = b.operator-(a);
		ac = c.operator-(a);
		//compute the normals
		//abPerp=(ACxAB)xAB=AB(AB.dot(AC))-AC(AB.dot(AB))
		abPerp = tripleProduct(ac, ab, ab);
		//acPerp=(ABxAC)xAC=AC(AC.dot(AB))-AB(AC.dot(AC))
		acPerp = tripleProduct(ab, ac, ac);

		//origin in the other side of AB
		if (abPerp.operator*(ao) > 0)
		{
			//remove vertex B
			simplex.erase(simplex.begin());
			//set new direction to abPerp
			d = abPerp;
		}
		//origin in the other side of AC
		else if (acPerp.operator*(ao) > 0)
		{
			//remove vertex C
			simplex.erase(simplex.begin() + 1);
			//set new direction to acPerp
			d = acPerp;
		}
		//origin in the triangle
		else
		{
			return true;
		}
	}

	//simplex is line segment
	else
	{
		b = simplex[0];
		//compute AB
		ab = b.operator-(a);
		//compute the normal
		abPerp = tripleProduct(ab, ao, ab);
		//set new direction to abPerp
		d = abPerp;
	}
	return false;
}

//vector axbxc=b(c.a)-a(c.b)
Vector SteerLib::GJK_EPA::tripleProduct(const Vector a, const Vector b, const Vector c)
{
	float ca = c.operator*(a);
	float cb = c.operator*(b);
	Vector abc = b.operator*(ca).operator-(a.operator*(cb));
	return abc;
}

Vector SteerLib::GJK_EPA::support(const std::vector<Vector>& _shapeA, const std::vector<Vector>& _shapeB, Vector& d)
{
	Vector p1, p2, p3;

	// get points on the dege of the shapes in opposite directions
	p1 = farthestPointInDirection(_shapeA, d);
	p2 = farthestPointInDirection(_shapeB, d.operator-());

	// Minkowski Difference
	p3 = p1.operator-(p2);
	return p3;
}

Vector SteerLib::GJK_EPA::farthestPointInDirection(const std::vector<Vector>& _shapeA, Vector& d)
{
	Vector farthestPoint = _shapeA[0];
	std::vector<Vector>::size_type l = _shapeA.size();

	//get farthest vertex in direction d
	for (unsigned i = 1; i < l; i++)
	{
		float a = d.operator*(_shapeA[i]);
		float b = d.operator*(farthestPoint);

		if (a > b)
		{
			farthestPoint = _shapeA[i];
		}
	}

	return farthestPoint;
}
/*
Vector SteerLib::GJK_EPA::centerOfPolygon(const std::vector<Vector>& _shapeA)
{
	//the number of vertices in shape A
	std::vector<Vector>::size_type l = _shapeA.size();

	Vector center;
	//center.zero();  //may useless

	//get center of A
	for (unsigned i = 0; i< l; i++)
	{
		center.operator+=(_shapeA[i]);
	}
	center.operator/=((float)l);

	return center;
}
*/