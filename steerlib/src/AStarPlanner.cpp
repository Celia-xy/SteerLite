//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <iostream>
#include <algorithm> 
#include <functional>
#include <math.h>
#include <limits>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

//set the preference of h and f value
// Manhattan: 1 for Manhattan distance, and 0 for Euclidian distance
// BIG_G: 1 for big g and 0 for small g (the preference when f values are equal)
// WEIGHT: set the weight of weighted A*, such as 1, 2, 4, 8
// DIFFERENT_DIAGONAL: 1 to consider diagonal cost as sqrt(2), and 0 to consider diagonal cost as 1
#define MANHATTAN 1
#define BIG_G 1
#define WEIGHT 1
#define DIFFERENT_DIAGONAL 1
 
//set weight for weighted A* : 1, 2, 4, 8
int weight = 8;

// set 1 if we consider all the costs to neighbors as 1
// set 2 if we consider diagonal costs as sqrt(2)
int dist_type = 2;

namespace SteerLib
{
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}

	bool AStarPlanner::canBeTraversed(int id, Util::Point current, std::vector<AStarNode>& neighborset)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost = gSpatialDatabase->getTraversalCost(index);

				if ((traversal_cost < COLLISION_COST) && (index != current_id)) {

					//get all avaliable neighbors
					neighborset.push_back(AStarNode(new AStarPlannerNode(index, 0.0f, 0.0f)));								
				}
			}
		}

		//check all avaliable neighbors when consider the radius of agent
		std::vector<Util::Point> neighbor_points;
		if (!neighborset.empty()) {
			bool left = false;
			bool right = false;
			bool up = false;
			bool down = false;

			for (AStarNode& neighbor_astar : neighborset) {

				Util::Point neighbor = getPointFromGridIndex(neighbor_astar->index());
				if (neighbor.x == current.x - 1 && neighbor.z == current.z)  left = true;
				if (neighbor.x == current.x + 1 && neighbor.z == current.z)  right = true;
				if (neighbor.x == current.x && neighbor.z == current.z - 1)  down = true;
				if (neighbor.x == current.x && neighbor.z == current.z + 1)  up = true;
			}

			//check all avaliable neighbors when consider the radius of agent
			//std::vector<Util::Point> neighbor_points;
			if (!neighborset.empty()) {
				bool left = true;
				bool right = true;
				bool up = true;
				bool down = true;

				for (AStarNode& neighbor_astar : neighborset) {

					Util::Point neighbor = getPointFromGridIndex(neighbor_astar->index());
					if (neighbor.x == current.x - 1 && neighbor.z == current.z)  left = false;
					if (neighbor.x == current.x + 1 && neighbor.z == current.z)  right = false;
					if (neighbor.x == current.x && neighbor.z == current.z - 1)  down = false;
					if (neighbor.x == current.x && neighbor.z == current.z + 1)  up = false;
				}
				int it = 0;
				while (it < neighborset.size()) {
			
					AStarNode neighbor_astar = neighborset.at(it);
					Util::Point neighbor = getPointFromGridIndex(neighbor_astar->index());
					//std::vector<AStarNode>::iterator it = std::find(neighborset.begin(), neighborset.end(), neighbor_astar);
					
					if (left || up) {
						if (neighbor.x == current.x - 1 && neighbor.z == current.z + 1) {
							neighborset.erase(neighborset.begin()+it);
							std::cout << 1 << std::endl;
							it--;
						}
					}
					if (left || down) {
						if (neighbor.x == current.x - 1 && neighbor.z == current.z - 1) {
							neighborset.erase(neighborset.begin() + it);

							std::cout << 2 << std::endl;
							it--;
						}
					}
					if (right || up) {
						if (neighbor.x == current.x + 1 && neighbor.z == current.z + 1) {
							neighborset.erase(neighborset.begin() + it);

							std::cout << 3 << std::endl;
							it--;
						}
					}
					if (right || down) {
						if (neighbor.x == current.x + 1 && neighbor.z == current.z - 1) {
							neighborset.erase(neighborset.begin() + it);
	
							std::cout << 4 << std::endl;
							it--;
						}
						
					}
					it++;
				}
			}
			
		}
		
		if (neighborset.empty()) {
			return false;
		}
		return true;
	}

	// calculate the distance between the point and its neighbor
	double AStarPlanner::dist_between(Util::Point point1, Util::Point point2) {		
		
		if (point1.x == point2.x || point1.z == point2.z)
			return 1;
		else if (DIFFERENT_DIAGONAL) {
			return sqrt(2);
		}
		else return 1;
	}


    // renew the f value of AstarNode
	void AStarPlanner::renewF(AStarNode& node_astar, Util::Point goal)
	{
		
        //calculate h value
		double h;
		Util::Point current_point = getPointFromGridIndex(node_astar->index());

		if (MANHATTAN) {
			h = abs(current_point.x - goal.x) + abs(current_point.z - goal.z);
		}
		else {
			h = pow(current_point.x - goal.x, 2) + pow(current_point.z - goal.z, 2);
			h = sqrt(h);
		}

		//calculate f value
		if (BIG_G) {
			//node_astar.f = weight * h + 0.999 * node_astar.g;
			node_astar->f(node_astar->g()*0.999 + WEIGHT * h);
		}
		else {
			//node_astar.f = weight * h + 1.001 * node_astar.g;
            node_astar->f(node_astar->g()*1.001 + WEIGHT * h);
		}
		return;
	}


	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	bool operator==(const AStarNode& point1, const AStarNode& point2) {
		return point1->index() == point2->index();
	}

	bool operator<(const AStarNode& point1, const AStarNode& point2) {
		if (point1->f() == point2->f()) {
			return point1->g() > point2->g();
		}
		else {
			return point1->f() < point2->f();
		}
	}

	void AStarPlanner::pop_min(std::vector<AStarNode>& list, AStarNode& current_astar) {
		std::vector<AStarNode>::iterator it = std::min_element(list.begin(), list.end());
		current_astar = *it;
		list.erase(it);
	}
	

	// implement A*
	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		// the openset and closedsest
		std::vector<AStarNode> openset;
		std::vector<AStarNode> closedset;

		// number of expanded nodes
		int num_expanded = 0;
		// total cost of the path
		double cost = 0.0;

		// the start point
		AStarNode start_astar(new AStarPlannerNode(gSpatialDatabase->getCellIndexFromLocation(start), 0, 0));
		openset.push_back(start_astar);

		// the index of destination
		AStarNode goal_astar(nullptr);
		int goal_index = gSpatialDatabase->getCellIndexFromLocation(goal);

		while (!openset.empty()) {
			// pop current point with smallest f value
			AStarNode current_astar(nullptr);
			pop_min(openset, current_astar);

			// find the path from start to goal
			if (current_astar->index() == goal_index) {

				goal_astar = current_astar;
				// track back for final path
				AStarNode pre_point_astar = goal_astar;
				Util::Point point = getPointFromGridIndex(pre_point_astar->index());
				while (pre_point_astar != nullptr) {

					Util::Point pre_point = getPointFromGridIndex(pre_point_astar->index());					
					agent_path.insert(agent_path.begin(), pre_point);
					pre_point_astar = pre_point_astar->parent();

					// compute cost of final path
					cost += dist_between(point, pre_point);			
					point = pre_point;
				}

				num_expanded = openset.size() + closedset.size();
				std::cout << "Number of expanded nodes: " << num_expanded << std::endl;
				std::cout << "The cost of final path: " << cost << std::endl;
				return true;
			}

			Util::Point current;
			gSpatialDatabase->getLocationFromIndex(current_astar->index(), current);
			// check all the neighbors of current point
			std::vector<AStarNode> neighborset;
			if (canBeTraversed(current_astar->index(), current, neighborset)) {

				for (AStarNode& neighbor_astar : neighborset) {

					int neighbor_index = neighbor_astar->index();
					Util::Point neighbor = getPointFromGridIndex(neighbor_index);
					neighbor_astar->g(current_astar->g() + dist_between(current, neighbor));
					renewF(neighbor_astar, goal);				
					std::vector<AStarNode>::iterator it = std::find(openset.begin(), openset.end(), neighbor_astar);

					// if in openset, change its g value to the smaller one and renew f
					if (it != openset.end()) {
						if (neighbor_astar->g() < (*it)->g()) {
							(*it)->g(neighbor_astar->g());
							(*it)->f(neighbor_astar->f());
							(*it)->parent(current_astar);
						}
					}
					else {
						// if not in closedset, insert it into openset
						if (std::find(closedset.begin(), closedset.end(), neighbor_astar) == closedset.end()) {
							neighbor_astar->parent(current_astar);
							openset.push_back(neighbor_astar);						
						}
					}				
				}
				closedset.push_back(current_astar);
				
			}
		}
		return false;
	}
}