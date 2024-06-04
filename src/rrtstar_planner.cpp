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
  max_iterations_ = 1000;

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

std::vector<int> RRTStar::findVerticesInsideCircle(double center_x, double center_y, double radius) {
    std::vector<int> vertices_inside_circle;

    RCLCPP_INFO(node_->get_logger(), "Radius of the circle: %.2f", radius);

    for (auto it = tree_.begin(); it != tree_.end(); ++it){
        double distance_squared = std::pow(it->x - center_x, 2) + std::pow(it->y - center_y, 2);
        if (distance_squared <= std::pow(radius, 2)) {
            vertices_inside_circle.push_back(std::distance(tree_.begin(), it));
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Number of vertices found within the circle: %lu", vertices_inside_circle.size());

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
    if (steps > 0){
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
    }
    return true;
}

double RRTStar::calculate_cost_from_start(const Vertex& vertex) {
    double total_cost = 0.0;

    const Vertex* cur_ver = &vertex;

    while (cur_ver != nullptr) {
        total_cost += cur_ver->cost;
        cur_ver = cur_ver->parent;
    }

    return total_cost;
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

  // Set up a random position generator
  calculateBallRadiusConstant();
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> x_dis(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInCellsX() * costmap_->getResolution());
  std::uniform_real_distribution<> y_dis(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInCellsY() * costmap_->getResolution());

  // Add start position to the tree
  tree_.clear();
  tree_.reserve(max_iterations_);
  Vertex start_vertex(start.pose.position.x, start.pose.position.y);
  start_vertex.cost = 0;
  tree_.emplace_back(start_vertex);
  
  // Create vertex for the end point
  Vertex end_vertex(goal.pose.position.x, goal.pose.position.y);
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = goal.pose.position.x;
  pose.pose.position.y = goal.pose.position.y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = goal.pose.orientation.x;
  pose.pose.orientation.y = goal.pose.orientation.y;
  pose.pose.orientation.z = goal.pose.orientation.z;
  pose.pose.orientation.w = 1.0;
  global_path.poses.insert(global_path.poses.begin(), pose);



  for (int i = 1; i <= max_iterations_-1; ++i) {
      // Generate a random point
      double rand_x = x_dis(gen);
      double rand_y = y_dis(gen);
      Vertex new_position(rand_x, rand_y);

      // Find nearest neighbor and assign its parent to new_position
      Vertex* nearest = nearest_neighbor(rand_x, rand_y);
      new_position.parent = nearest;
      new_position.cost = calculate_distance(nearest->x, nearest->y, new_position);

      if (connectible(*nearest, new_position)) {
          // Perform rewire operation
          double ball_radius = calculateBallRadius(tree_.size(), 2, 2.0);

          std::vector<int> vertices_inside_circle = findVerticesInsideCircle(new_position.x, new_position.y, ball_radius);
          tree_.emplace_back(new_position);

          RCLCPP_INFO( node_->get_logger(), "New vertex x: %.4f", new_position.x);
          RCLCPP_INFO( node_->get_logger(), "New vertex y: %.4f", new_position.y);

          double total_cost_for_new_position = calculate_cost_from_start(tree_[i]);

          int num_of_rewiring = 0;

          // Check if there is a better route from start towards the new position
          for (size_t j = 0; j < vertices_inside_circle.size(); ++j){
            int index = vertices_inside_circle[j];
            double potential_cost = calculate_cost_from_start(tree_[index]) + calculate_distance(new_position.x, new_position.y, tree_[index]);
            if (potential_cost < total_cost_for_new_position && connectible(tree_[i], tree_[index])){
              tree_[i].parent = &tree_[index];
              tree_[i].cost = calculate_distance(new_position.x, new_position.y, tree_[index]);
              total_cost_for_new_position = potential_cost;
              num_of_rewiring+=1;
            }
          }

          // Check if any existing vertex may benefit from being connected by the new position
          for (size_t j = 0; j < vertices_inside_circle.size(); ++j){
            int index = vertices_inside_circle[j];
            double current_cost = calculate_cost_from_start(tree_[index]);
            double potential_cost = calculate_cost_from_start(tree_[i]) + calculate_distance(new_position.x, new_position.y, tree_[index]);
            if (potential_cost < total_cost_for_new_position && connectible(tree_[i], tree_[index])){
              tree_[index].parent = &tree_[i];
              tree_[index].cost = calculate_distance(new_position.x, new_position.y, tree_[index]);
              num_of_rewiring+=1;
              RCLCPP_INFO( node_->get_logger(), "I'm going to rewire a node");
            }
          }

        RCLCPP_INFO( node_->get_logger(), "Iteration %d", i);
      }
      else{
        i -= 1;
      }
  }

  // Find optimal way to the goal

  double ball_radius = 2*calculateBallRadius(tree_.size(), 2, 2.0);
  std::vector<int> vertices_inside_circle = findVerticesInsideCircle(goal.pose.position.x, goal.pose.position.y, ball_radius);
  while (true){

    double min_cost = std::numeric_limits<double>::infinity();

    for (size_t j = 0; j < vertices_inside_circle.size(); ++j){
      int index = vertices_inside_circle[j];
      double potential_cost = calculate_cost_from_start(tree_[index]) + calculate_distance(goal.pose.position.x, goal.pose.position.y, tree_[index]);
      if (potential_cost < min_cost && connectible(end_vertex, tree_[index])){
        end_vertex.parent = &tree_[index];
        end_vertex.cost = calculate_distance(goal.pose.position.x, goal.pose.position.y, tree_[index]);
        min_cost = potential_cost;
      }
    }      

    if (min_cost < 10000){
    tree_.emplace_back(end_vertex);

    RCLCPP_INFO( node_->get_logger(), "Emplaced end_vertex");

    int k = 0;

    for (auto it = tree_.begin(); it != tree_.end(); ++it){
      RCLCPP_INFO( node_->get_logger(), "%d, %.5f, %.5f, %.5f, %p, %p", k, it->x, it->y, it->cost, &(*it), it->parent);
      k++;
    }


    Vertex* cur_ver = &end_vertex;
    while (cur_ver) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = cur_ver->x;
        pose.pose.position.y = cur_ver->y;
        pose.pose.position.z = 0.0;

        global_path.poses.insert(global_path.poses.begin(), pose);

        RCLCPP_INFO( node_->get_logger(), "Parent of current vertex: %p", cur_ver->parent);
        //Add waypoints between the random points
        if (cur_ver->parent != nullptr){
          double steps = std::ceil(std::hypot(cur_ver->x - cur_ver->parent->x, cur_ver->y - cur_ver->parent->y) * 10);
          double x_increment = (cur_ver->parent->x - cur_ver->x) / steps;
          double y_increment = (cur_ver->parent->y - cur_ver->y) / steps;

          double x = cur_ver->x;
          double y = cur_ver->y;

          for (int i = 0; i < steps-1; ++i) {
            x += x_increment;
            y += y_increment;
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;

            global_path.poses.insert(global_path.poses.begin(), pose);
          }

        }          
        cur_ver = cur_ver->parent;
      }
      break;
      }
      if (ball_radius >100){
        break;
      }
    ball_radius += 0.5;
    vertices_inside_circle = findVerticesInsideCircle(goal.pose.position.x, goal.pose.position.y, ball_radius);
  }
  return global_path;
}

}  // namespace nav2_rrtstar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrtstar_planner::RRTStar, nav2_core::GlobalPlanner)