#include <vector> 
#include <algorithm> 

#include "obstacles/GJK_EPA.h"

using namespace Util;

// Helper declarations
Util::Vector getCenter(const std::vector<Util::Vector> &shape);
Util::Vector getFarthestPointInDirection(const std::vector<Util::Vector> &shape, const Util::Vector &direction);
Util::Vector getSupport(const std::vector<Util::Vector> &shapeA, const std::vector<Util::Vector> &shapeB, const Util::Vector &direction);
Util::Vector getNewDirection(const std::vector<Util::Vector> &simplex);

SteerLib::GJK_EPA::GJK_EPA()
{
	// GJK constructor
	// test build
}

// Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	return false; // There is no collision
}

// Construct simplex and find one that contains the origin
bool constructSimplex(const std::vector<Vector> shapeA, const std::vector<Vector> shapeB)
{
	std::vector<Vector> simplex;  // Simplex that stores Minkowski points

	// Get the direction by A - B
	Vector d = getCenter(shapeA) - getCenter(shapeB);
	simplex.push_back(getSupport(shapeA, shapeB, d));
	d = -d; // negate d

	while (true)
	{
		simplex.push_back(getSupport(shapeA, shapeB, d));
		// Checkthat last simplex point added was passed the origin
		if (dot(simplex.back(), d) <= 0)
		{
			return false;
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
