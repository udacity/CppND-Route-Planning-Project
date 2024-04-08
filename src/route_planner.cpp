#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &model.FindClosestNode(start_x, start_x);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}

// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // handle the unlikely case that the node is null by returning the maximum possible for a float
    if (node == nullptr) {
        return std::numeric_limits<float>::max();
    }
    return node->distance(*this->end_node);
}


// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Guard against the unlikely case that the node is null
    if (current_node == nullptr) {
        return;
    }
    // Populate current_node.neighbors vector with all the neighbors
    current_node->FindNeighbors();
    // For each node in current_node.neighbors, set the parent, the h_value, the g_value
    for (auto neighbor: current_node->neighbors) {
        if (neighbor->visited) {
            continue;
        }
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        // Add the neighbor to open_list and set the node's visited attribute to true
        this->open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}


// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value
    std::sort(this->open_list.begin(), this->open_list.end(), [](RouteModel::Node *a, RouteModel::Node *b){
        return a->g_value + a->h_value > b->g_value + b->h_value;
    });
    // Remove the lowest sum node from the open_list and return a pointer to it
    RouteModel::Node *lowest = this->open_list.back();
    this->open_list.pop_back();
    return lowest;
}

// constructPath is a recursive function that walks the node path and calculates the distance traveled.
// Using a recursive function call allows us to return the final path without having to reverse it.
void constructPath(std::vector<RouteModel::Node> &path, RouteModel::Node *current_node, float &distance) {
    // Terminating condition
    if (current_node == nullptr) {
        return;
    }
    RouteModel::Node *parent_node = current_node->parent;
    // Recursive function call
    constructPath(path, parent_node, distance);

    path.push_back(*current_node);
    if (current_node->parent != nullptr) {
        distance += current_node->distance(*parent_node);
    }
}

// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node != nullptr) {
        path_found.push_back(*current_node);
        RouteModel::Node *parent_node = current_node->parent;
        if (parent_node != nullptr) {
            distance = distance + current_node->distance(*parent_node);
        }
        current_node = parent_node;
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    // The returned vector should be in the correct order: the start node should be the first element
    std::reverse(path_found.begin(), path_found.end());
    return path_found;
}


// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;
    // Add all of the neighbors of the start node to the open_list
    AddNeighbors(current_node);
    start_node->visited = true;

    while(! this->open_list.empty()) {
        // Sort the open_list and return the next node
        current_node = this->NextNode();

        if (current_node == end_node) {
            // Store the final path in the m_Model.path
            this->m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // Add all of the neighbors of the current node to the open_list
        AddNeighbors(current_node);
    }
}