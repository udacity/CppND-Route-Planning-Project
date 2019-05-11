#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) 
{
    //1/ Convert all OSM Nodes to RouteModel::Node to be able to perform A*
    //2/ Store them in RouteModel.m_Nodes
    int index = 0;
    for (Model::Node currNode : this->Nodes())
        this->m_Nodes.push_back(RouteModel::Node(index++, this, currNode));

}