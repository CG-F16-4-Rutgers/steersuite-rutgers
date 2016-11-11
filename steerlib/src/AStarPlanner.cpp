//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}
	


	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;

		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	//Populates a list of neighbors only if a neighbor is valid//
	bool AStarPlanner::AddNodeValid(AStarPlannerNode* start, std::vector<AStarPlannerNode*>, Util::Point origin) {
		int dbIndex = gSpatialDatabase->getCellIndexFromLocation(origin);

		if (canBeTraversed(origin)) {
			AStarPlannerNode* node = new AStarPlannerNode(point, double(0), double(0), double(0), start);
			neighbors.push_back(node);
			return true;
		}

		return false;
	}


	std::vector<AStarPlannerNode*> AStarPlanner::getNeighbors(AStarPlannerNode* origin) {
	/*  
	
		     	SIDE VIEW
									
	    x-1, z+1 | z+1 | x+1,z+1
	    ----------------------		
	       x-1   |  0  |   x+1 
	    ----------------------      
	     x-1,z-1 | z-1 | x+1,z-1     
		
																*/
		std::vector<AStarPlannerNode*> neighbors;
		
	//starting from x axis left to right//
		AddNodeValid(origin, neighbors, Util::Point(origin->point.x - 1, 0, origin->point.z + 1);
		AddNodeValid(origin, neighbors, Util::Point(origin->point.x - 1, 0, origin->point.z);
		AddNodeValid(origin, neighbors, Util::Point(origin->point.x - 1, 0, origin->point.z - 1);
		AddNodeValid(origin, neighbors, Util::Point(origin->point.x, 0, origin->point.z + 1);
		AddNodeValid(origin, neighbors, Util::Point(origin->point.x, 0, origin->point.z - 1);
		AddNodeValid(origin, neighbors, Util::Point(origin->point.x + 1, 0, origin->point.z - 1);
		AddNodeValid(origin, neighbors, Util::Point(origin->point.x + 1, 0, origin->point.z);
		AddNodeValid(origin, neighbors, Util::Point(origin->point.x + 1, 0, origin->point.z + 1);

		return neigbors;
	}


	//Calculates Euclidian Distance//
	double AStarPlanner::euclidian_heuristic(Util::Point start, Util:Point end) {
		//calculate variables for distance formula//
		double e_x = (start.x - end.x)*(start.x - end.x);
		double e_y = (start.y - end.y)*(start.y - end.y);
		double e_x = (start.z - end.z)*(start.z - end.z);
		double euclidian_distance = std::sqrt(e_x+e_y+e_z);
		return euclidian_distance;
	}

	//A function to identify the node with minimum f-value//
	int AStarPlanner::minimumF(std::vector<AStarPlannerNode*> Directory) {
		double min = 500000000;
		int position = -1;
		for (int i = 0; i < Directory.size(); i++) {
			if (Directory[i]-> <= min) {
				min = Directory[i];
				position = i;
			}
		}
		return position;
	}

	std::vector<Util::Point> AStarPlanner::trace(AStarPlannerNode* node) {
		std::vector<Util::Point> trace;
		AStarPlannerNode* temp = node;
		trace.push_back(temp -> point);
		while (temp -> parent != NULL) {
			temp = temp -> parent;
			trace.push_back(temp -> point);
		}
		std::vector<Util::Point> traceTemp;
		for (int i = 0; i < trace.size(); i++) {
			
			traceTemp.push_back(trace[trace.size() - 1 - i]);
		}
		return traceTemp;
	}
	
	bool AStarPlanner::weightedAStar(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path) {
		gSpatialDatabase = _gSpatialDatabase;

		std::vector<AStarPlannerNode*> open_list;
		std::vector<AStarPlannerNode*> closed_list;

		AStarPlannerNode* root = new AStarPlannerNode(start, double(0), euclidean_heuristic(goal, start), euclidean_heuristic(goal, start), NULL);
		open_list.push_back(root);

		while (!open_list.empty()) {

			int indexOfQ = indexWithLeastF(open_list);
			AStarPlannerNode* q = open_list[indexOfQ];

			open_list.erase(open_list.begin() + indexOfQ);

			std::vector<AStarPlannerNode*> successors = getNeighbors(q);
			
			for (int i = 0; i < successors.size(); i++) {
				if (successors[i] -> point == goal) {
					agent_path = trace(successors[i]);
					return true;
				}

				successors[i] -> g = q -> g + euclidean_heuristic(q -> point, successors[i] -> point);
				
				successors[i] -> h = euclidean_heuristic(goal, successors[i] -> point);

				successors[i] -> f = successors[i] -> g + w*successors[i] -> h;


				bool skip = false;
				for (int j = 0; j < open_list.size(); j++) {
					// open list
					if (open_list[j] -> point == successors[i] -> point && open_list[j] -> f < successors[i] -> f) {
						skip = true;
					}
				}

				for (int j = 0; j < closed_list.size(); j++) {
					// closed list
					if (closed_list[j] -> point == successors[i] -> point && closed_list[j] -> f < successors[i] -> f) {
						skip = true;
					}
				}

				if (!skip) {
					open_list.push_back(successors[i]);
				}
			}
			closed_list.push_back(q);
		}

		return false;
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout<<"\nIn A*";

		return false;
	}
}
