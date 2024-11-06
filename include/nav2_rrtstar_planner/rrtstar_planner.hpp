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