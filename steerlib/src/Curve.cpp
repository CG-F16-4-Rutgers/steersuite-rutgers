//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;


// Function declarations 
bool sortControlPointsByTime(const CurvePoint& p1, const CurvePoint& p2);
bool ifSameTime(const CurvePoint& p1, const CurvePoint& p2);

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve
	

	// Draw nothing if there are less than two control points
	if (!checkRobust()) { return; }

	Point startPoint, nextPoint;
	for (int i = 0; i < controlPoints.size(); ++i) 
	{
		if (i == controlPoints.size() - 1) // Draw the from current position to last point
		{
			DrawLib::glColor(curveColor);
			DrawLib::drawLine(startPoint, controlPoints[i].position, curveColor, curveThickness);
			break;
		}

		startPoint = controlPoints[i].position;

		for (int j = (controlPoints[i]).time; j <= (controlPoints[i + 1]).time; j += window)   // Step through controlPoints by using window as time
		{
			calculatePoint(nextPoint, j);
			DrawLib::glColor(curveColor);
			DrawLib::drawLine(startPoint, nextPoint, curveColor, curveThickness);

			startPoint = nextPoint;
		}
	}

#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	std::sort(controlPoints.begin(), controlPoints.end(), sortControlPointsByTime);
	controlPoints.erase(std::unique(controlPoints.begin(), controlPoints.end(), ifSameTime), controlPoints.end()); // Remove if we encounter a new CP with the same time as a prior CP
}

// Sort helper functions
bool ifSameTime(const CurvePoint& p1, const CurvePoint& p2)
{
	return p1.time == p2.time;
}

bool sortControlPointsByTime(const CurvePoint& p1, const CurvePoint& p2)
{
	return p1.time < p2.time;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	if (controlPoints.size() < 2) { return false; }     // Curve needs at least two points

	else return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	// if time is greater or equal to the last control point's time then we've reached past the end
	if (time >= controlPoints.back().time)
	{
		return false;
	}

	// Set nextPoint to index of next control point
	for (unsigned int i = 0; i < controlPoints.size(); ++i)
	{
		if (controlPoints[i].time > time)
		{
			nextPoint = i;
			break;
		}
	}

	return true; // when nextPoint is found
}


// current
// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	if (time < controlPoints[nextPoint - 1].time || time > controlPoints[nextPoint].time)
	{
		return Point(0, 0, 0);
	}

	unsigned int prevPoint = nextPoint - 1;

	Point prevPointPos = controlPoints[prevPoint].position;          // Position of previous control point
	Point nextPointPos = controlPoints[nextPoint].position;          // Position of next control point
	Vector prevPointTangent = controlPoints[nextPoint - 1].tangent;  // Tangent of previous control point
	Vector nextPointTangent = controlPoints[nextPoint].tangent;      // Tangent of next control point

	// Time variables
	float prevPointTime = controlPoints[prevPoint].time;  // t0
	float nextPointTime = controlPoints[nextPoint].time;  // t1
	float elapsedTime = time - prevPointTime;             // t
	float intervalTime = (time - prevPointTime) / (nextPointTime - prevPointTime); 	// Normalize time from prevPoint to nextPoint (e.g (t - t0) / (t1 - t0) )

	// Hermite Blending function (expanded all terms)
	Point a = prevPointPos * (2 * pow(intervalTime, 3) - (3 * pow(intervalTime, 2)) + 1);
	Point b = nextPointPos * (-2 * pow(intervalTime, 3) + (3 * pow(intervalTime, 2)));
	Vector c = prevPointTangent * ( ( pow(elapsedTime, 3) / pow(nextPointTime - prevPointTime, 2) ) + ( -2 * pow(elapsedTime, 2) / (nextPointTime - prevPointTime) ) + elapsedTime );
	Vector d = nextPointTangent * ( ( pow(elapsedTime, 3) / pow(nextPointTime - prevPointTime, 2) ) - ( pow(elapsedTime, 2) / (nextPointTime - prevPointTime) ) );

	Point newPosition = a + b + c + d;
	
	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{

	unsigned int prevPoint = nextPoint - 1;

	Point prevPointPos = controlPoints[prevPoint].position;          // Position of previous control point
	Point nextPointPos = controlPoints[nextPoint].position;          // Position of next control point
	Vector prevPointTangent = controlPoints[nextPoint - 1].tangent;  // Tangent of previous control point
	Vector nextPointTangent = controlPoints[nextPoint].tangent;      // Tangent of next control point

	// Time variables
	float prevPointTime = controlPoints[prevPoint].time;  // t0
	float nextPointTime = controlPoints[nextPoint].time;  // t1
	float elapsedTime = time - prevPointTime;             // t
	float intervalTime = (time - prevPointTime) / (nextPointTime - prevPointTime); 	// Normalize time from prevPoint to nextPoint (e.g (t - t0) / (t1 - t0) )

	Vector si, sn;
	// Set start boundary control point tangent
	if (nextPoint == 1)
	{
		float t0 = controlPoints[0].time;       
		float t1 = controlPoints[1].time;
		float t2 = controlPoints[2].time;

		Point y0 = controlPoints[0].position;
		Point y1 = controlPoints[1].position;
		Point y2 = controlPoints[2].position;

		si = ((t2 - t0) / (t2 - t1) * (y1 - y0) / (t1 - t0) - (t1 - t0) / (t2 - t1) * (y2 - y0) / (t2 - t0));
	} 
	
	// Set nth from [1, n-1] control point tangents
	else
	{
		float t_0 = controlPoints[nextPoint - 2].time;      // t_i-1
		float t_1 = controlPoints[nextPoint - 1].time;      // t_i
		float t_2 = controlPoints[nextPoint].time;          // t_i+1

		Point y_0 = controlPoints[nextPoint - 2].position;  // y_i-1
		Point y_1 = controlPoints[nextPoint - 1].position;  // y_i
		Point y_2 = controlPoints[nextPoint].position;      // y_i+1

		si = ((t_1 - t_0) / (t_2 - t_0) * (y_2 - y_1) / (t_2 - t_1)) + ((t_2 - t_1) / (t_2 - t_0) * (y_1 - y_0) / (t_1 - t_0));
	}

	// Set end control point tangent 
	if (nextPoint == controlPoints.size() - 1)
	{
		int size = controlPoints.size();

		float t_0 = controlPoints[size - 3].time;           // t_n-3
		float t_1 = controlPoints[size - 2].time;           // t_n-2
		float t_2 = controlPoints[size - 1].time;           // t_n-1

		Point y_0 = controlPoints[size - 3].position;       // y_n-3
		Point y_1 = controlPoints[size - 2].position;       // y_n-2
		Point y_2 = controlPoints[size - 1].position;       // y_n-1

		//sn = ((t_2 - t_0) / (t_2 - t_1) * (y_1 - y_0) / (t_1 - t_0) - (t_1 - t_0) / (t_2 - t_1) * (y_2 - y_0) / (t_2 - t_0)); // incorrect blend for end point, should be symmetrical to start boundary blend function
		sn = ((t_2 - t_0) / (t_1 - t_0) * (y_2 - y_1) / (t_2 - t_1) - (t_2 - t_1) / (t_1 - t_0) * (y_2 - y_0) / (t_2 - t_0));
	}
	// Set next control point tangent to be one point ahead
	else
	{
		float t_0 = controlPoints[nextPoint - 1].time;      // t_i
		float t_1 = controlPoints[nextPoint].time;          // t_i+1
		float t_2 = controlPoints[nextPoint + 1].time;      // t_i+2

		Point y_0 = controlPoints[nextPoint - 1].position;  // y_i
		Point y_1 = controlPoints[nextPoint].position;      // y_i+1
		Point y_2 = controlPoints[nextPoint + 1].position;  // y_i+2

		sn = ((t_1 - t_0) / (t_2 - t_0) * (y_2 - y_1) / (t_2 - t_1)) + ((t_2 - t_1) / (t_2 - t_0) * (y_1 - y_0) / (t_1 - t_0));
	}

	// Blending function (expanded all terms)
	Point a = prevPointPos * (2 * pow(intervalTime, 3) - (3 * pow(intervalTime, 2)) + 1);
	Point b = nextPointPos * (-2 * pow(intervalTime, 3) + (3 * pow(intervalTime, 2)));
	Vector c = si * ( ( pow(elapsedTime, 3) / pow(nextPointTime - prevPointTime, 2) ) + ( -2 * pow(elapsedTime, 2) / (nextPointTime - prevPointTime) ) + elapsedTime );
	Vector d = sn * ( ( pow(elapsedTime, 3) / pow(nextPointTime - prevPointTime, 2) ) - ( pow(elapsedTime, 2) / (nextPointTime - prevPointTime) ) );

	Point newPosition = a + b + c + d;

	// Return result
	return newPosition;
}