#pragma once

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

      private:
        // Add private Node variables and methods here.
    };
    
    // Add public RouteModel variables and methods here.
  
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.

  private:
    // Add private RouteModel variables and methods here.

};
