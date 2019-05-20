#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        // Add public Node variables and methods here.
        
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
      	float distance(Node n)  const {
        	float dist = std::sqrt(std::pow((x - n.x), 2) + std::pow((y - n.y), 2) );
            return dist;
        }
        bool operator< (const Node& node) const {return h_value + g_value < node.h_value + node.g_value;} //meant for sorting, doesn't work because vec of pointers?!
        void FindNeighbors();
        void FindNeighbors_v2_better();
      	Node* parent {nullptr};
      	float g_value {0.0};
     	  float h_value {std::numeric_limits<float>::max()};
      	bool visited {false};
        std::vector<Node*> neighbors {};

      	
      private:
        // Add private Node variables and methods here.
        
      	int index;
        RouteModel * parent_model = nullptr;
    	Node*  FindNeighbor(std::vector<int> node_indices);
    };
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);  
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.
	  std::vector<Node> &SNodes() {return m_Nodes;}
  	auto &GetNodeToRoadMap() {return node_to_road;}
  	RouteModel::Node& FindClosestNode(float x, float y);

  private:
    // Add private RouteModel variables and methods here.
	  std::vector<Node> m_Nodes {};
  	std::unordered_map<int, std::vector<const Model::Road*>> node_to_road {};
    void CreateNodeToRoadHashmap();
};

#endif