#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);

    //A* Search
    void AStarSearch();

    //Get found solution distance
    float GetDistance() const { return distance; }

  private:
    // OSM model augmented to performed A*
    RouteModel &m_Model;

    //Start and end Nodes of the search
    RouteModel::Node *start_node, *end_node;

    //Found route distance
    float distance;

    //List of open Nodes used uding search
    std::vector<RouteModel::Node *> open_list;

    //Rebuild path from last Node, from parent to parent until reaching start node
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *current_node);

    //Compute H Value of specific Node
    //  (Heuristic is distance to end node)
    float CalculateHValue(const RouteModel::Node node);

    //From the list of open Nodes, find the one with lowest F-value
    //  F is H(heuristic) + G(current weight)
    RouteModel::Node * NextNode();

    //Add new neigbor to node's list (and update its h/g/parent)
    void AddNeighbors(RouteModel::Node *newNode);
};
