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
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.01));
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

double RRTStar::calculate_distance(double x, double y, Vertex vertex)
{
  return sqrt(pow(vertex.x - x, 2) + pow(vertex.y - y, 2));
}

Vertex* RRTStar::nearest_neighbor(double x, double y)
{
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


bool RRTStar::connectible(Vertex start, Vertex end) {
    double resolution = interpolation_resolution_;
    double steps = ceil(std::hypot(end.x - start.x, end.y - start.y) / resolution);
    double x_increment = (end.x - start.x) / steps;
    double y_increment = (end.y - start.y) / steps;

    double x = start.x, y = start.y;
    for (auto i = 0; i < steps; ++i) {
        unsigned int mx, my;
        if (!costmap_->worldToMap(x, y, mx, my)) return false;
        // RCLCPP_INFO(node_->get_logger(), "x %f", x);
        // RCLCPP_INFO(node_->get_logger(), "mx %u", mx); 
        // the line below need some changes because it's not checking properly
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
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  //calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  // setting up random position generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<>x_dis(-3.0, 3.0);
  std::uniform_real_distribution<>y_dis(-3.0, 3.0);

  // adding start position to the tree
  tree_.clear();
  tree_.shrink_to_fit();
  tree_.reserve(max_iterations_);
  Vertex start_vertex(start.pose.position.x, start.pose.position.y);
  tree_.emplace_back(start_vertex);
  Vertex end_vertex(goal.pose.position.x, goal.pose.position.y);

  for (int i = 0; i < max_iterations_; ++i) {
      double rand_x = x_dis(gen);
      double rand_y = y_dis(gen);
      Vertex new_position(rand_x, rand_y);

      // Find nearest neighbor and assign its parent to new_position
      Vertex* parent_of_new_position = nearest_neighbor(rand_x, rand_y);

      new_position.parent = parent_of_new_position;

      if (connectible(*(parent_of_new_position), new_position)) {
          tree_.emplace_back(new_position);
      }

      if (connectible(new_position, end_vertex)) {
          end_vertex.parent = &new_position;
          tree_.emplace_back(end_vertex);

          Vertex cur_ver = end_vertex;
          while (true) {
              geometry_msgs::msg::PoseStamped pose;
              pose.pose.position.x = cur_ver.x;
              pose.pose.position.y = cur_ver.y;
              pose.pose.position.z = 0.0;

              global_path.poses.insert(global_path.poses.begin(), pose);

              // Break if there is no parent or if the parent is already visited
              if (cur_ver.parent == nullptr) {
                  break;
              }
              cur_ver = *(cur_ver.parent);
          }
          break;
      }
      
      // geometry_msgs::msg::PoseStamped pose;
      // pose.pose.position.x = start.pose.position.x + x_increment * i;
      // pose.pose.position.y = start.pose.position.y + y_increment * i;
      // pose.pose.position.z = 0.0;
      // pose.pose.orientation.x = 0.0;
      // pose.pose.orientation.y = 0.0;
      // pose.pose.orientation.z = 0.0;
      // pose.pose.orientation.w = 1.0;
      // //pose.header.stamp = node_->now();
      // //pose.header.frame_id = global_frame_; 
      // global_path.poses.push_back(pose);
    }

  // geometry_msgs::msg::PoseStamped goal_pose = goal;
  // goal_pose.header.stamp = node_->now();
  // goal_pose.header.frame_id = global_frame_;
  // global_path.poses.push_back(goal_pose);
	
  return global_path;
}

}  // namespace nav2_rrtstar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrtstar_planner::RRTStar, nav2_core::GlobalPlanner)
