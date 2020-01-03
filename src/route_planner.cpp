#include "route_planner.h"
#include <algorithm>

void PrintNodeInfo(const RouteModel::Node *node){

    std::cout << "Node: X: " << node->x << "| Y: " << node->y << std::endl;
    std::cout << "g: " << node->g_value << "| h: " << node->h_value << "| f: " << node->g_value + node->h_value << std::endl;
}

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

    return node->distance(*end_node);

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    current_node->FindNeighbors();
    //neibhbors is a vecotr of node pointers
    for (RouteModel::Node* nodePtr : current_node->neighbors){
        if(nodePtr->visited == false){
            nodePtr->parent = current_node;
            nodePtr->h_value = CalculateHValue(nodePtr);
            nodePtr->g_value = current_node->g_value + current_node->distance(*nodePtr);
            nodePtr->visited = true;
            this->open_list.push_back(nodePtr);
        }
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool Compare(RouteModel::Node *n1, RouteModel::Node *n2){
    auto f1 = n1->g_value + n1->h_value;
    auto f2 = n2->g_value + n2->h_value;
    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {

    std::sort(this->open_list.begin(), this->open_list.end(), Compare);
    RouteModel::Node* lowest_sum_ptr = this->open_list.back();
    this->open_list.pop_back();
    return lowest_sum_ptr;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
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

    // TODO: Implement your solution here.
    RouteModel::Node* next_node_ptr = current_node;

    while (next_node_ptr != start_node)
    {
        distance += next_node_ptr->distance(*next_node_ptr->parent);
        path_found.insert(path_found.begin(), *next_node_ptr);
        next_node_ptr = next_node_ptr->parent;
    }
    path_found.insert(path_found.begin(), *next_node_ptr);
 
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here. 
    this->start_node->visited = true;
    AddNeighbors(this->start_node);

    while(this->open_list.size() > 0){
        
        current_node = NextNode();
        if(current_node == end_node){
            this->m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);
    }
}