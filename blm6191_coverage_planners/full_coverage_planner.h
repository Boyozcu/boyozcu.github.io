#ifndef FULL_COVERAGE_PLANNER_H
#define FULL_COVERAGE_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <string>
#include <algorithm> // std::min, std::max

namespace blm6191_coverage_planners {

class FullCoveragePlanner : public nav_core::BaseGlobalPlanner {
public:
    FullCoveragePlanner();
    FullCoveragePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    void polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    bool isPointInPolygon(const geometry_msgs::Point& point, const std::vector<geometry_msgs::Point>& polygon_points_geom);
    bool isObstacle(double world_x, double world_y);
    // İmza doğru (5 parametre)
    bool findClearPathSegment(const geometry_msgs::Point& start_p, const geometry_msgs::Point& end_p,
                               std::vector<geometry_msgs::PoseStamped>& segment_plan, double orientation_yaw,
                               const std::vector<geometry_msgs::Point>& current_polygon_geom);

    void generateBoustrophedonPath(const std::vector<geometry_msgs::Point>& polygon_corners,
                                   const geometry_msgs::PoseStamped& start_pose,
                                   std::vector<geometry_msgs::PoseStamped>& plan);

    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    ros::Publisher plan_pub_;
    std::string frame_id_;

    double robot_radius_;
    double sweep_spacing_factor_;
    double path_point_distance_;

    std::vector<geometry_msgs::Point32> polygon_corners_msg_;
    bool use_param_poly_;
    ros::Subscriber polygon_sub_;

    double p1x_, p1y_, p2x_, p2y_, p3x_, p3y_, p4x_, p4y_;
};

} // namespace blm6191_coverage_planners

#endif // FULL_COVERAGE_PLANNER_H
