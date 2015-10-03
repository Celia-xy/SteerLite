//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"
#include <math.h>

using namespace Util;

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

	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;

	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function drawCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================
	*/

	Point startPoint, endPoint;
	const int numberOfControl = controlPoints.size();
	const int timeOfControl = controlPoints[numberOfControl - 1].time;

	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
	{
		return;
	}
	
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	if (!calculatePoint(startPoint, 1)) { return; }
	
	for (int i = 0; i < (timeOfControl / window); i++)
	{
		if (!calculatePoint(endPoint, (i+1)*window)) { return; }
		
		DrawLib::drawLine(startPoint, endPoint, curveColor, curveThickness);
		startPoint = endPoint;
	}
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function sortControlPoints is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================
	*/
	struct timeAscending
	{
		inline bool operator() (const CurvePoint& point1, const CurvePoint& point2)
		{
			// sort by time
			return(point1.time < point2.time);
		}
	};

	std::sort(controlPoints.begin(), controlPoints.end(), timeAscending());
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	// Assume that the agent start position has been added into the controlPoints vector
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
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
	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function checkRobust is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================
	*/

	// At least two points are needed to make a curve
	if (controlPoints.size()-2>=0)  return true;
	else return false;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function findTimeInterval is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================
	*/
	const int numberOfControl = controlPoints.size();

	if (time <= controlPoints[0].time | time > controlPoints[numberOfControl - 1].time) 
	{
		return false;
	}

	for (int i = 0; i < numberOfControl; i++)
	{
		if (controlPoints[i].time < time & controlPoints[i + 1].time >= time) 
		{ 
			nextPoint = i + 1; 
			return true;
		}
	}

}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;
	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useHermiteCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================
	*/

	// Calculate time interval, and normal time required for later curve calculations
	float timeInterval = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	float deltaTime = time - controlPoints[nextPoint - 1].time;
	// Calculate position at t = time on Hermite curve
	
	newPosition =	controlPoints[nextPoint - 1].position.operator*(2 * pow(deltaTime / timeInterval, 3) -
					3 * pow(deltaTime / timeInterval, 2) + 1).operator+(
					controlPoints[nextPoint].position.operator*(-2 * pow(deltaTime / timeInterval, 3) +
					3 * pow(deltaTime / timeInterval, 2))).operator+(
					controlPoints[nextPoint - 1].tangent.operator*(pow(deltaTime / timeInterval, 2) -
					2 * deltaTime / timeInterval + 1).operator*(deltaTime)).operator+(
					controlPoints[nextPoint].tangent.operator*(pow(deltaTime / timeInterval, 2) -
					1 * deltaTime / timeInterval).operator*(deltaTime));
	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useCatmullCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================
	*/

	// Calculate time interval, and normal time required for later curve calculations
	float timeInterval = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	float deltaTime = time - controlPoints[nextPoint - 1].time;
	const int numberOfControl = controlPoints.size();

	// general form
	 Vector s0, si, sn, si_plus_1;
	
	// judge whether there are controlPoints on same time 
	float deltaTime0, deltaTime1, deltaTime2;
	if (nextPoint != 1) 
	{
		deltaTime0 = controlPoints[nextPoint - 1].time - controlPoints[nextPoint - 2].time;
	}
	
	deltaTime1 = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	
	if (nextPoint != numberOfControl - 1) 
	{
		deltaTime2 = controlPoints[nextPoint + 1].time - controlPoints[nextPoint].time;
	}

	if (deltaTime0 == 0) {
		deltaTime0 = 50;
	}
	if (deltaTime1 == 0) {
		deltaTime1 = 50;
	}
	if (deltaTime2 == 0) {
		deltaTime2 = 50;
	}

	// judge whether it is in the first or the last interval. If so, change the corresponding tangent 
	if (nextPoint == 1) 
	{ 
		si = controlPoints[1].position.operator-(controlPoints[0].position).operator*
			(((deltaTime1 + deltaTime2) / deltaTime2) / deltaTime1).operator-
			(controlPoints[2].position.operator-(controlPoints[0].position).operator*
				((deltaTime1 / deltaTime2) / (deltaTime1 + deltaTime2)));

		si_plus_1 = controlPoints[nextPoint + 1].position.operator-(controlPoints[nextPoint].position).operator*
			((deltaTime1 / (deltaTime1 + deltaTime2)) / deltaTime2).operator+
			(controlPoints[nextPoint].position.operator-(controlPoints[nextPoint - 1].position).operator*
				((deltaTime2 / (deltaTime1 + deltaTime2)) / deltaTime1));

	}
	else if (nextPoint == numberOfControl - 1) 
	{
		si = controlPoints[nextPoint].position.operator-(controlPoints[nextPoint - 1].position).operator*
			((deltaTime0 / (deltaTime1 + deltaTime0)) / deltaTime1).operator+
			(controlPoints[nextPoint - 1].position.operator-(controlPoints[nextPoint - 2].position).operator*
				((deltaTime1 / (deltaTime1 + deltaTime0)) / deltaTime0));

		si_plus_1 = controlPoints[numberOfControl - 1].position.operator-(controlPoints[numberOfControl - 2].position).operator*
			((deltaTime1 + deltaTime0) / deltaTime0 / deltaTime1).operator-
			(controlPoints[numberOfControl - 1].position.operator-(controlPoints[numberOfControl - 3].position).operator*
				(deltaTime1 / deltaTime0 / (deltaTime1 + deltaTime0)));
	}
	else 
	{
		// calculate each time interval si and si+1
		si = controlPoints[nextPoint].position.operator-(controlPoints[nextPoint - 1].position).operator*
			((deltaTime0 / (deltaTime1 + deltaTime0)) / deltaTime1).operator+
			(controlPoints[nextPoint - 1].position.operator-(controlPoints[nextPoint - 2].position).operator*
				((deltaTime1 / (deltaTime1 + deltaTime0)) / deltaTime0));
		si_plus_1 = controlPoints[nextPoint + 1].position.operator-(controlPoints[nextPoint].position).operator*
			((deltaTime1 / (deltaTime1 + deltaTime2)) / deltaTime2).operator+
			(controlPoints[nextPoint].position.operator-(controlPoints[nextPoint - 1].position).operator*
				((deltaTime2 / (deltaTime1 + deltaTime2)) / deltaTime1));
	}

	// Calculate position at t = time on catmull curve
	newPosition =	controlPoints[nextPoint - 1].position.operator*(2 * pow(deltaTime / timeInterval, 3) -
					3 * pow(deltaTime / timeInterval, 2) + 1).operator+(
					controlPoints[nextPoint].position.operator*(-2 * pow(deltaTime / timeInterval, 3) +
					3 * pow(deltaTime / timeInterval, 2))).operator+(
					si.operator*(pow(deltaTime / timeInterval, 2) -
					2 * deltaTime / timeInterval + 1).operator*(deltaTime)).operator+(
					si_plus_1.operator*(pow(deltaTime / timeInterval, 2) -
					1 * deltaTime / timeInterval).operator*(deltaTime));
	// Return result
	
	return newPosition;

}