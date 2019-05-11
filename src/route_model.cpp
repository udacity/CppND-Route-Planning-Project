#include "route_model.h"
#include <iostream>

//Distance computation from another Node
float RouteModel::Node::distance(const Model::Node &otherNode) const
{
    return std::sqrt(std::pow(this->x - otherNode.x, 2.f) + std::pow(this->y - otherNode.y, 2.f));
}

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) 
{
    //1/ Convert all OSM Nodes to RouteModel::Node to be able to perform A*
    //2/ Store them in RouteModel.m_Nodes
    int index = 0;
    for (Model::Node currNode : this->Nodes())
        this->m_Nodes.push_back(RouteModel::Node(index++, this, currNode));

}