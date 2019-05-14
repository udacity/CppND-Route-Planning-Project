#include "route_planner.h"
#include <algorithm>

//========================================================
// RoutePlanner Class

RoutePlanner::RoutePlanner(RouteModel &model, 
                           float start_x, float start_y, 
                           float end_x, float end_y) :
    m_Model(model)
{
    //Find pointers to closest nodes to start/end position
    //Note: coordinates are changed to percent
    start_node = &m_Model.FindClosestNode(start_x * 0.01, start_y * 0.01);
    end_node   = &m_Model.FindClosestNode(end_x * 0.01, end_y * 0.01);
}

//---------------
// Public methods

//A* Search
void RoutePlanner::AStarSearch()
{
    //Temporary stub, direct path from start to end
    end_node->parent = start_node;
    m_Model.path = ConstructFinalPath(end_node);
}

//----------------
// Private methods

//Rebuild path from last Node, from parent to parent until reaching start node
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    //empty path and 0 distance
    std::vector<RouteModel::Node> pathFound;
    distance = 0.f;

    //Add current node to begining of the path and update total distance
    while (current_node->parent != nullptr)
    {
        pathFound.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    pathFound.push_back(*current_node);

    //Apply scale to distance
    distance *= m_Model.MetricScale();

    return pathFound;
}

//Compute H Value of specific Node
//  (Heuristic is distance to end node)
float RoutePlanner::CalculateHValue(const RouteModel::Node node)
{
    return node.distance(*end_node);
}

//Helper function used by sort, to compare the F values of two nodes
bool CompareF(const RouteModel::Node *a, RouteModel::Node *b) 
{
  return a->g_value + a->h_value > b->g_value + b->h_value ; 
}

//From the list of open Nodes, find the one with lowest F-value
//  F is H(heuristic) + G(current weight)
RouteModel::Node * RoutePlanner::NextNode()
{
    //sort the vector of Nodes
    std::sort(open_list.begin(), open_list.end(), CompareF);

    //Get smallest F value node, and pop-it from list
    RouteModel::Node *result = open_list.back();
    open_list.pop_back();
    return result;
}

//Add new neigbor to node's list (and update its h/g/parent)
void RoutePlanner::AddNeighbors(RouteModel::Node *newNode)
{
    //find all connected (non-visited) nodes
    newNode->FindNeighbors();

    //update found neigbors
    for (auto currNeighbors : newNode->neighbors)
    {
        //link node to parent
        currNeighbors->parent = newNode;

        //update g (cost to reach neighbor) 
        currNeighbors->g_value = newNode->g_value + newNode->distance(*currNeighbors);

        //update heuristic (distance to target)
        currNeighbors->h_value = CalculateHValue(*currNeighbors);

        //add neigbor to open list
        open_list.push_back(currNeighbors);

        //mark as visited to avoid re-use
        currNeighbors->visited = true;
    }
}
