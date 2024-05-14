/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include <random>
#include <vector>
#include <limits>

#include "nav2_rrtstar_planner/rrtstar_planner.hpp"
namespace nav2_rrtstar_planner
{

void RRTStar::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  max_iterations_ = 2000;

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.01));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void RRTStar::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void RRTStar::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void RRTStar::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void RRTStar::calculateBallRadiusConstant() {
    double resolution = costmap_->getResolution();
    double cellArea = resolution * resolution;
    unsigned int numFreeCells = 0;

    for (unsigned int x = 0; x < costmap_->getSizeInCellsX(); x++) {
        for (unsigned int y = 0; y < costmap_->getSizeInCellsY(); y++) {
            if (costmap_->getCost(x, y) == nav2_costmap_2d::FREE_SPACE) {
                numFreeCells++;
            }
        }
    }

    double freeVolume = cellArea * numFreeCells; 
    int dimensions = 2;  
    double vUnitBall = M_PI;  
    ball_radius_constant_ = 2.0 * (1 + 1.0 / dimensions) * std::pow((freeVolume / vUnitBall), (1.0 / dimensions));
    RCLCPP_INFO(node_->get_logger(), "Ball Radius Constant: %f", ball_radius_constant_);
}

double RRTStar::calculateBallRadius(int tree_size, int dimensions, double max_connection_distance) {
    double term1 = (ball_radius_constant_ * std::log(tree_size)) / tree_size;
    double term2 = std::pow(term1, 1.0 / dimensions);
    return std::min(term2, max_connection_distance);
}

std::vector<Vertex*> RRTStar::findVerticesInsideCircle(double center_x, double center_y, double radius) {
    std::vector<Vertex*> vertices_inside_circle;

    for (auto& vertex : tree_) {
        double distance_squared = std::pow(vertex.x - center_x, 2) + std::pow(vertex.y - center_y, 2);
        if (distance_squared <= std::pow(radius, 2)) {
            vertices_inside_circle.push_back(&vertex);
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Number of vertices found within the circle: %d", vertices_inside_circle.size());

    return vertices_inside_circle;
}

double RRTStar::calculate_distance(double x, double y, const Vertex& vertex) {
    return std::sqrt(std::pow(vertex.x - x, 2) + std::pow(vertex.y - y, 2));
}

Vertex* RRTStar::nearest_neighbor(double x, double y) {
    Vertex* nearest_vertex = nullptr;
    double min_distance = std::numeric_limits<double>::infinity();

    for (auto& vertex : tree_) {
        double dist = calculate_distance(x, y, vertex);
        if (dist < min_distance) {
            min_distance = dist;
            nearest_vertex = &vertex;
        }
    }
    return nearest_vertex;
}

bool RRTStar::connectible(const Vertex& start, const Vertex& end) {
    double resolution = interpolation_resolution_;
    double steps = std::ceil(std::hypot(end.x - start.x, end.y - start.y) / resolution);
    double x_increment = (end.x - start.x) / steps;
    double y_increment = (end.y - start.y) / steps;

    double x = start.x, y = start.y;
    for (int i = 0; i < steps; ++i) {
        unsigned int mx, my;
        if (!costmap_->worldToMap(x, y, mx, my)) return false;
        if (costmap_->getCost(mx, my) != nav2_costmap_2d::FREE_SPACE) return false;
        x += x_increment;
        y += y_increment;
    }
    return true;
}

nav_msgs::msg::Path RRTStar::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only accept start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only accept goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // setting up random position generator
  calculateBallRadiusConstant();
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> x_dis(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInCellsX() * costmap_->getResolution());
  std::uniform_real_distribution<> y_dis(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInCellsY() * costmap_->getResolution());

  // adding start position to the tree
  tree_.clear();
  tree_.reserve(max_iterations_);
  Vertex start_vertex(start.pose.position.x, start.pose.position.y);
  start_vertex.cost = 0;
  tree_.emplace_back(start_vertex);
  Vertex end_vertex(goal.pose.position.x, goal.pose.position.y);

  for (int i = 0; i < max_iterations_; ++i) {
      double rand_x = x_dis(gen);
      double rand_y = y_dis(gen);
      Vertex new_position(rand_x, rand_y);

      // Find nearest neighbor and assign its parent to new_position
      Vertex* nearest = nearest_neighbor(rand_x, rand_y);
      new_position.parent = nearest;
      new_position.cost = nearest->cost + calculate_distance(nearest->x, nearest->y, new_position);

      if (connectible(*nearest, new_position)) {
          tree_.emplace_back(new_position);

          // Perform rewire operation
          double ball_radius = calculateBallRadius(tree_.size(), 2, interpolation_resolution_);
          std::vector<Vertex*> vertices_inside_circle = findVerticesInsideCircle(new_position.x, new_position.y, ball_radius);
          for (auto vertex_ptr : vertices_inside_circle) {
              Vertex& vertex = *vertex_ptr;
              double potential_cost = new_position.cost + calculate_distance(new_position.x, new_position.y, vertex);
              if (potential_cost < vertex.cost && connectible(new_position, vertex)) {
                  vertex.parent = &new_position;
                  vertex.cost = potential_cost;
              }
          }

          if (connectible(new_position, end_vertex)) {
              end_vertex.parent = &new_position;
              end_vertex.cost = new_position.cost + calculate_distance(new_position.x, new_position.y, end_vertex);
              tree_.emplace_back(end_vertex);

              Vertex* cur_ver = &end_vertex;
              while (cur_ver) {
                  geometry_msgs::msg::PoseStamped pose;
                  pose.pose.position.x = cur_ver->x;
                  pose.pose.position.y = cur_ver->y;
                  pose.pose.position.z = 0.0;

                  global_path.poses.insert(global_path.poses.begin(), pose);
                  cur_ver = cur_ver->parent;
              }

              break;
          }
      }
  }

  return global_path;
}

}  // namespace nav2_rrtstar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrtstar_planner::RRTStar, nav2_core::GlobalPlanner)
