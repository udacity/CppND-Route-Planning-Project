#pragma once

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>
#include <vector>

class RouteModel : public Model {

  public:

    //A* Enabled Nodes
    class Node : public Model::Node {
      public:
        //previous Node in chain
        RouteModel::Node* parent = nullptr;
        //Heuristics distance and cost from start
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.f;
        //Was node already visited
        bool visited = false;
        //List of contiguous nodes
        std::vector<RouteModel::Node *> neighbors;
        
        //Node constructor extending OSM nodes
        Node(){}
        Node(int idx, RouteModel* search_model, Model::Node node) : 
            Model::Node(node), 
            parent_model(search_model), 
            index(idx) {}

        //Distance computation from another Node
        float distance(const Model::Node &otherNode) const;

        //Populate the neighbors vector of this node 
        //   (closest nodes on all roads this node is on)
        void FindNeighbors();
      
      private:
        //Node index in list & parent reference
        int index;
        RouteModel* parent_model = nullptr;

        //Find closest Node in a list from current
        RouteModel::Node* FindNeighbor(std::vector<int> node_indices);
    };
    
    //Route model constructor
    RouteModel(const std::vector<std::byte> &xml);

    //Reverse map getter (used for test)
    auto &GetNodeToRoadMap() { return node_to_road; }

    //Path that is found by the A* search.  
    std::vector<Node> path; 

    //Getter for the private nodes vector
    std::vector<RouteModel::Node> &SNodes() { return m_Nodes; }

  private:
    //Building of the reverse map
    void CreateNodeToRoadHashmap(void);

    //List of A* enabled Nodes
    std::vector<RouteModel::Node> m_Nodes;

    //Reverse map of NodeID -> Road to help the graph search
    std::unordered_map <int, std::vector<const Model::Road *>> node_to_road;
};
