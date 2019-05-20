#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
	for (int idx=0; idx < this->Nodes().size(); idx++) {
    	m_Nodes.push_back(Node(idx, this, this->Nodes().at(idx)));
    }
    CreateNodeToRoadHashmap();
}
                                                 
void RouteModel::CreateNodeToRoadHashmap() {
	for (const Model::Road &road : Roads()){
  		if (road.type != Model::Road::Type::Footway){
        	for (int idx : Ways()[road.way].nodes){
            	if (node_to_road.find(idx) == node_to_road.end()) {
                	node_to_road[idx] = std::vector<const Model::Road*> {};
                }
                node_to_road[idx].push_back(&road);
            }
        }
    }
  
}
                                                 
RouteModel::Node*  RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
	Node *closest_node = nullptr;
  	Node node;
  	for (int node_idx : node_indices) {
    	node = parent_model->SNodes()[node_idx];
      	if (this->distance(node)>0 && !node.visited) {
        	if (closest_node == nullptr || (this->distance(node) < this->distance(*closest_node))){
            	closest_node = &(parent_model->SNodes()[node_idx]);
            }
        }
    }
    return closest_node;
}

void RouteModel::Node::FindNeighbors(){
	for (auto &road : parent_model->node_to_road[this->index]) {
    	RouteModel::Node* roadnode = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (roadnode) {
        	neighbors.push_back(roadnode);
        }
    }
}

//the method below taken from damiens solution in the forums (https://github.com/dbecad/CppND-Route-Planning-Project)
//not used in submitted version, because it fails the tests (because resulting route is better ;-) )
void RouteModel::Node::FindNeighbors_v2_better() 
{
    //For each roads going through the node (from reverese map)
    for (const Model::Road *roadPtr : parent_model->GetNodeToRoadMap()[index])
    {
        //Find two connected neigbors to current node
        RouteModel::Node *prevNode = nullptr, *nextNode = nullptr;
        std::vector<int> nodeList = parent_model->Ways()[roadPtr->way].nodes;
        for (int currIndex = 0; currIndex < nodeList.size(); currIndex++)
        {
            //if current node is the one searched stop the search
            if (nodeList[currIndex] == index)
            {
                //check if there one after non visited and add it
                if (currIndex != nodeList.size() - 1)
                    if (parent_model->SNodes()[nodeList[currIndex + 1]].visited == false)
                        nextNode = &parent_model->SNodes()[nodeList[currIndex + 1]];
                break;
            }
            else
            {
                //keep that as the previous node if it was not visited (or clear it otherwise)
                if (parent_model->SNodes()[nodeList[currIndex]].visited == false)
                    prevNode = &parent_model->SNodes()[nodeList[currIndex]];
                else
                    prevNode = nullptr;
            }            
        } 

        //Add prev/next if available
        if (nextNode != nullptr)
            neighbors.push_back(nextNode);
        if (prevNode != nullptr)
            neighbors.push_back(prevNode);
    }
}

RouteModel::Node &RouteModel::FindClosestNode(float x, float y){
	Node temp = Node();
  	temp.x = x;
  	temp.y = y;
  	float min_dist = std::numeric_limits<float>::max();
  	int closest_idx {};

  	for (auto &road : Roads()){
    	if (road.type != Model::Road::Type::Footway) {
        	for (auto node_idx : Ways()[road.way].nodes) {
            		auto dist = temp.distance(SNodes()[node_idx]);
              	  	if (dist < min_dist){
                  		closest_idx = node_idx;
                  		min_dist = dist;
                    }
            }
        }
    }
  	return SNodes()[closest_idx];
}
