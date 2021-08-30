#include "route_planner.h"
#include <iostream>
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
start_x *= 0.01;
start_y *= 0.01;
end_x *= 0.01;
end_y *= 0.01;

this->start_node = &m_Model.FindClosestNode(start_x, start_y);
this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
current_node->FindNeighbors();
for(auto node : current_node->neighbors){
node->parent = current_node;
node->h_value = CalculateHValue(node);
node->g_value = current_node->g_value + current_node->distance(*node);
node->visited = true;
open_list.push_back(node);
}
}


bool compare(const RouteModel::Node* node1, const RouteModel::Node* node2){
return node1->g_value + node1->h_value > node2->g_value + node2->h_value;
}

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(open_list.begin(),open_list.end(), compare); 
    auto *lowest_sum=open_list.back();
    open_list.pop_back();
    return lowest_sum;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
// Create path_found vector
distance = 0.0f;
std::vector<RouteModel::Node> path_found;
// auto current_node=NextNode();
while(current_node!=start_node){
distance+=current_node->distance(*(current_node->parent));
path_found.push_back(*current_node);
  current_node=current_node->parent;
}
path_found.push_back(*current_node);
distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
std::reverse(path_found.begin(), path_found.end());
return path_found;
}

void RoutePlanner::AStarSearch() {
start_node->visited = true;
open_list.push_back(start_node);
  RouteModel::Node *current_node = start_node;;
  while (open_list.size() > 0) {
current_node = NextNode();
if (current_node->distance(*end_node) == 0) {
m_Model.path = ConstructFinalPath(current_node);
return;
}
AddNeighbors(current_node);
}
}