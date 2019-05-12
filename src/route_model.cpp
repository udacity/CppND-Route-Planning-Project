#include "route_model.h"
#include "model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
  int counter = 0;
  for (const auto &node : this->Nodes()) {
    m_Nodes.push_back(Node(counter, this, node));
    counter++;
  }
  CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap() {
  for (const auto &road : Roads()) {
    if (road.type == Model::Road::Type::Footway) {
      continue;
    }
    
    for (int node_idx : Ways()[road.way].nodes) {
      if (node_to_road.find(node_idx) == node_to_road.end()) {
        node_to_road[node_idx] = std::vector<const Model::Road *>{};
      }
      node_to_road[node_idx].push_back(&road);
    }
  }
}

RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
  Node *closest_node = nullptr;
  float closest_distance = std::numeric_limits<float>::max();
  for(int node_idx : node_indices) {
    Node &node = parent_model->SNodes()[node_idx];
    auto d = this->distance(node);
    if (node.visited || d == 0) {
      continue;
    }
    
    if (d < closest_distance) {
      closest_node = &node;
      closest_distance = d;
    }
  }
  return closest_node;
}

void RouteModel::Node::FindNeighbors() {
  for (auto& road : parent_model->node_to_road[this->index]) {
    RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    if (new_neighbor) {  
      this->neighbors.emplace_back(new_neighbor);
    }
  }
}

RouteModel::Node& RouteModel::FindClosestNode(float x, float y) {
  Node input;
  input.x = x;
  input.y = y;
  
  float min_dist = std::numeric_limits<float>::max();
  int closest_idx = 0;
  for (auto& road : Roads()) {
    if (road.type == Model::Road::Type::Footway) {
      continue;
    }
    
    for (int node_idx : Ways()[road.way].nodes) {
      float dist = input.distance(SNodes()[node_idx]);
      if (dist < min_dist) {
        closest_idx = node_idx;
        min_dist = dist;
      }
    }
  }
  return SNodes()[closest_idx];
}
