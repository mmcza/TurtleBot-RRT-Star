// Copyright 2024 Marcin Czajka, Daria Kubacka
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Portions of this file are derived from nav2_straightline_planner
// (https://github.com/ros-navigation/navigation2_tutorials), which is
// distributed under the following license:
//
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

#ifndef NAV2_RRTSTAR_PLANNER__RRTSTAR_PLANNER_HPP_
#define NAV2_RRTSTAR_PLANNER__RRTSTAR_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_rrtstar_planner {

struct Vertex {
    double x, y, cost;
    Vertex* parent;
    Vertex(double x_val, double y_val, Vertex* p = nullptr, double travel_distance = 0) :
        x(x_val), y(y_val), parent(p), cost(travel_distance) {}
};

class RRTStar : public nav2_core::GlobalPlanner {
public:
    RRTStar() = default;
    ~RRTStar() override = default;

    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                   std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                   const geometry_msgs::msg::PoseStamped& goal) override;

protected:
    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D* costmap_;
    std::string global_frame_;
    std::string name_;
    int max_iterations_;
    double interpolation_resolution_;
    std::vector<std::unique_ptr<Vertex>> tree_;
    double ball_radius_constant_;

    double calculate_distance(double x, double y, const Vertex& vertex);
    Vertex* nearest_neighbor(double x, double y);
    bool connectible(const Vertex& start, const Vertex& end);
    void calculateBallRadiusConstant();
    double calculateBallRadius(int tree_size, int dimensions, double max_connection_distance);
    std::vector<int> findVerticesInsideCircle(double center_x, double center_y, double radius);
    double calculate_cost_from_start(const Vertex& vertex);
};

}  // namespace nav2_rrtstar_planner

#endif  // NAV2_RRTSTAR_PLANNER__RRTSTAR_PLANNER_HPP_