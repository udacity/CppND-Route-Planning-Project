#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);

    //Getters

    //Get found solution distance
    float GetDistance() { return distance; }

  private:
    // OSM model augmented to performed A*
    RouteModel &m_Model;

    //Start and end Nodes of the search
    RouteModel::Node start_node, end_node;

    //Found route distance
    float distance;
};
