#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

  	start_node = &(m_Model.FindClosestNode(start_x, start_y));
  	end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    auto g = current_node->g_value;
    for(auto neighbor : current_node->neighbors)
    {
        neighbor->parent = current_node;
        neighbor->g_value = g+ current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor) ;
        neighbor->visited = true;
        open_list.push_back(neighbor);
    }

}

bool Compare(RouteModel::Node const *a, RouteModel::Node const *b) {
  return (a->g_value + a->h_value) > (b->g_value + b->h_value); 
}
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    auto lowest = open_list.back();
    open_list.pop_back();
    return lowest; 
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    while(current_node->parent != nullptr)
    {
      	path_found.insert(path_found.begin(), *current_node);
        auto parent_node = current_node->parent;
        distance+= current_node->distance(*parent_node);
        current_node = parent_node;
    }
    path_found.insert(path_found.begin(), *start_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
  	open_list.push_back(start_node);
    while (open_list.size() > 0)
    {
        current_node = NextNode();
        if(current_node == end_node){
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        else
        {
            AddNeighbors(current_node);
        }
    }
}