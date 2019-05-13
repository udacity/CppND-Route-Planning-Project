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
