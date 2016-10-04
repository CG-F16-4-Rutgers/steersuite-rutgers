#include <vector> 
#include <algorithm> 

#include "obstacles/GJK_EPA.h"

using namespace Util;

// Edges contain an index, edge distance, and edge normal, which is used to find MTV in EPA
struct Edge
{
public:
	int index;
	float distance;
	Vector normal;
};

// Helper declarations
Util::Vector getCenter(const std::vector<Util::Vector> &shape);
Util::Vector getFarthestPointInDirection(const std::vector<Util::Vector> &shape, const Util::Vector &direction);
Util::Vector getSupport(const std::vector<Util::Vector> &shapeA, const std::vector<Util::Vector> &shapeB, const Util::Vector &direction);
Util::Vector getNewDirection(const std::vector<Util::Vector> &simplex);
bool containsOrigin(std::vector<Vector> &simplex, Vector &direction);

// EPA Helpers
void getEPA(float &penetrationDepth, Vector &penetrationVector, std::vector<Vector> &simplex, const std::vector<Vector> &shapeA, const std::vector<Vector> &shapeB);
Edge getClosestEdge(std::vector<Vector> &simplex);

SteerLib::GJK_EPA::GJK_EPA()
{
	// GJK constructor
	// test build
}

// Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Vector> simplex;  // Simplex that stores Minkowski points								 
	Vector d = getCenter(_shapeA) - getCenter(_shapeB);  // Get the direction by A - B
	simplex.push_back(getSupport(_shapeA, _shapeB, d));
	d = -d; // negate d

	while (true)
	{
		simplex.push_back(getSupport(_shapeA, _shapeB, d));
		// Check that last simplex point added was passed the origin
		if (dot(simplex.back(), d) <= 0)
		{
			// The last simplex point has not gone past the origin using the current direction
			return false;
		}

		else // current simplex could potentially contain origin
		{
			// Check if simplex does contain origin
			if (containsOrigin(simplex, d))
			{
				// Since we found the simplex containing origin, we perform EPA to get the penetration information
				getEPA(return_penetration_depth, return_penetration_vector, simplex, _shapeA, _shapeB);
				return true; // Objects intersect
			}
			else
			{
				// Find a new direction that points to origin
				d = getNewDirection(simplex);
			}
		}
	}
}

// Get shape center using average method 
Vector getCenter(const std::vector<Vector> &shape)
{
	if (shape.size() == 0)
	{
		return Vector(0, 0, 0);
	}

	Vector sum(0, 0, 0);
	// Sum up all vertices
	for (int i = 0; i < shape.size(); ++i)
	{
		sum += shape[i];
	}

	return (sum / shape.size()); // Return the center of the shape
}


Vector getFarthestPointInDirection(const std::vector<Vector> &shape, const Vector &direction)
{
	// Choose any point to start
	Vector farthest = shape[0];

	float maxDP = dot(shape[0], direction); // Start with the dot prod of first vertex
	float currDP;
	for (int i = 1; i < shape.size(); ++i)
	{
		currDP = dot(shape[i], direction);
		if (currDP > maxDP)
		{
			// Store new max dot product and new farthest point
			maxDP = currDP;
			farthest = shape[i];
		}
	}
	return farthest;
}

// Build simplex by using a support on Minkowski difference
Vector getSupport(const std::vector<Vector> &shapeA, const std::vector<Vector> &shapeB, const Vector &direction)
{
	Vector farthestA = getFarthestPointInDirection(shapeA,  direction);
	Vector farthestB = getFarthestPointInDirection(shapeB, -direction);

	Vector support = farthestA - farthestB; // Minkowski difference

	return support;
}

// Generate origin containing simplex by choosing a new direction towards origin
Vector getNewDirection(const std::vector<Vector> &simplex)
{ 
	// We know that the latest points are more likely to contain the origin, else the algorithm would've exited
	Vector a = simplex[simplex.size() - 1];
	Vector b = simplex[simplex.size() - 2];

	Vector ab = b - a;
	Vector a0 = Vector(0, 0, 0) - a;  // Direction from A to origin

	// Get new direction by taking cross product
	Vector d = cross(cross(ab, a0), ab);
	return d;
}

// Check if the simplex contains origin
bool containsOrigin(std::vector<Vector> &simplex, Vector &direction)
{
	// b at index 0 
	// c at index 1
	// a at index 2 (or back)

	Vector a = simplex.back();
	Vector a0 = Vector(0, 0, 0) - a;

	// Triangle
	if (simplex.size() == 3)
	{
		Vector b = simplex[0];
		Vector c = simplex[1];
		Vector ab = b - a;     // a points to b
		Vector ac = c - a;     // a points to c

		// compute normals on ab and ac
		Vector perp_ab = cross(cross(ac, ab), ab); // perp_ab = (ab x ac) x ac
		Vector perp_ac = cross(cross(ab, ac), ac); // perp_ac = (ac x ab) x ab

		if (dot(perp_ab, a0) > 0)
		{
			simplex.erase(simplex.begin() + 1); // erase point c
			direction = perp_ab;
		}
		else
		{
			if (dot(perp_ac, a0) > 0)
			{
				simplex.erase(simplex.begin());
				direction = perp_ac;
			}
			else
			{
				return true;
			}
		}
	}
	else // Dealing with line segment instead of a triangle, so find new direction to get simplex point c
	{		
		Vector b = simplex[0];
		Vector ab = b - a;
		Vector perp_ab = cross(cross(ab, a0), ab);
		direction = perp_ab;
	}
	return false;
}

void getEPA(float &penetrationDepth, Vector &penetrationVector, std::vector<Vector> &simplex, const std::vector<Vector> &shapeA, const std::vector<Vector> &shapeB)
{
	while (true)
	{
		Edge edge = getClosestEdge(simplex); 
		Vector support = getSupport(shapeA, shapeB, edge.normal); // Get new support using edge normal

		float distance = dot(support, edge.normal);
		if (distance - edge.distance < 0.0001) // if less than some tolerance then we can assume that we can no longer expand simplex
		{
			penetrationVector = edge.normal;
			penetrationDepth = distance;
			break;
		}		
		else
		{
			// Insert new point into simplex
			simplex.insert(simplex.begin() + edge.index, support);
		}
	}
}

Edge getClosestEdge(std::vector<Vector> &simplex)
{
	Edge edge;
	edge.distance = 0;

	float closestDistance = FLT_MAX; // Start with the furthest float distance

	for (int i = 0; i < simplex.size(); ++i)
	{
		// Get the next simplex point to use 
		int j = i + 1;
		if (j == simplex.size()) // If we've reached the last simplex point, direct j back to beginning
		{
			j = 0;
		}
		
		Vector a = simplex[i];
		Vector b = simplex[j];

		Vector edgeVector = b - a; // Get the edge vector 
		Vector normal = cross(cross(edgeVector, a), edgeVector);
		normal = normalize(normal); // normalize the edge normal

		float distance = dot(normal, a); // Distance from origin to edge
		
		if (distance < closestDistance)
		{
			edge.index = j;
			edge.distance = distance;
			edge.normal = normal;
		}
	}
	return edge;
}
