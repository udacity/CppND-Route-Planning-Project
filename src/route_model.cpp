#include "route_model.h"
#include <iostream>
using std::cout;
using std::endl;
using std::vector;

//Distance computation from another Node
float RouteModel::Node::distance(const Model::Node &otherNode) const
{
    return std::sqrt(std::pow(this->x - otherNode.x, 2.f) + std::pow(this->y - otherNode.y, 2.f));
}

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) 
{
    //Convert all OSM Nodes to RouteModel::Node to be able to perform A*
    //Then store them in RouteModel.m_Nodes
    int index = 0;
    for (Model::Node currNode : this->Nodes())
        this->m_Nodes.push_back(RouteModel::Node(index++, this, currNode));

    //Build the reverse map from NodeIdx to Road*
    CreateNodeToRoadHashmap();
}

//Building of the reverse map
void RouteModel::CreateNodeToRoadHashmap(void)
{
    //For each roads of the model (Note: use reference to store address in map)
    for (const Model::Road &currRoad : Roads())
    {
        //Build reverse map only for non-footway roads
        if (currRoad.type != Model::Road::Type::Footway)
        {
            //Iterate over all nodes of current road's way
            for (int currNodeIdx : Ways()[currRoad.way].nodes)
            {
                //When node index is not yet present in list create an empty vector 
                if (node_to_road.find(currNodeIdx) == node_to_road.end())
                    node_to_road[currNodeIdx] = vector<const Model::Road *> ();

                //Add current Road pointer to node index entry
                node_to_road[currNodeIdx].push_back(&currRoad);
            }
        } 
    }
}
