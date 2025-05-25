#ifndef FULL_COVERAGE_PLANNER_H
#define FULL_COVERAGE_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_global_planner.h>
#include <vector>
#include <queue>
#include <map>
#include <string>
#include <set>

namespace blm6191_coverage_planners {

struct Node {
    double x, y;
    double g_cost, h_cost, f_cost;
    Node* parent;
    
    Node() : x(0.0), y(0.0), g_cost(0.0), h_cost(0.0), f_cost(0.0), parent(nullptr) {}
    
    Node(double x_, double y_, double g_ = 0, double h_ = 0) 
        : x(x_), y(y_), g_cost(g_), h_cost(h_), f_cost(g_ + h_), parent(nullptr) {}
    
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

// Obstacle region structure for detailed coverage
struct ObstacleRegion {
    std::vector<geometry_msgs::Point> contour_points;
    geometry_msgs::Point center;
    double max_radius;
    std::vector<std::vector<geometry_msgs::Point>> sub_regions; // Erişilebilir alt bölgeler
};

class FullCoveragePlanner : public nav_core::BaseGlobalPlanner {
public:
    FullCoveragePlanner();
    FullCoveragePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    ~FullCoveragePlanner() = default;
    
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    
    // Basic parameters
    double robot_radius_;
    double sweep_spacing_factor_;
    double path_point_distance_;
    std::string frame_id_;
    bool use_param_poly_;
    
    // Polygon parameters
    double p1x_, p1y_, p2x_, p2y_, p3x_, p3y_, p4x_, p4y_;
    std::vector<geometry_msgs::Point32> polygon_corners_msg_;
    
    // Enhanced obstacle avoidance parameters
    double safety_margin_;
    double max_obstacle_distance_;
    int max_detour_attempts_;
    double adaptive_spacing_factor_;
    bool use_dynamic_spacing_;
    double obstacle_check_resolution_;
    double line_of_sight_resolution_;
    
    // Coverage around obstacles parameters
    bool enable_obstacle_coverage_;
    double obstacle_coverage_radius_;
    double min_coverage_area_;
    int coverage_sweep_directions_;
    
    // Publishers and subscribers
    ros::Publisher plan_pub_;
    ros::Publisher obstacle_regions_pub_;
    ros::Subscriber polygon_sub_;
    
    // Core functions
    void polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    void generateBoustrophedonPath(const std::vector<geometry_msgs::Point>& poly_corners_geom,
                                   const geometry_msgs::PoseStamped& start_pose,
                                   std::vector<geometry_msgs::PoseStamped>& plan);
    
    // Enhanced obstacle avoidance functions
    bool isObstacle(double world_x, double world_y);
    bool isObstacleWithMargin(double world_x, double world_y, double margin = 0.0);
    bool isPointInPolygon(const geometry_msgs::Point& p, const std::vector<geometry_msgs::Point>& poly_points);
    bool isPointSafe(double world_x, double world_y, double safety_radius);
    
    // Obstacle region detection and coverage
    std::vector<ObstacleRegion> detectObstacleRegions(const std::vector<geometry_msgs::Point>& polygon);
    std::vector<geometry_msgs::Point> findAccessibleAreasAroundObstacle(
        const geometry_msgs::Point& obstacle_center, 
        double search_radius,
        const std::vector<geometry_msgs::Point>& polygon);
    void generateObstacleCoverage(
        const ObstacleRegion& obstacle_region,
        const std::vector<geometry_msgs::Point>& polygon,
        std::vector<geometry_msgs::PoseStamped>& coverage_plan);
    
    // Coverage pattern generation
    std::vector<geometry_msgs::PoseStamped> generateRadialCoverage(
        const geometry_msgs::Point& center,
        const std::vector<geometry_msgs::Point>& accessible_area,
        const std::vector<geometry_msgs::Point>& polygon);
    std::vector<geometry_msgs::PoseStamped> generateSpiralCoverage(
        const geometry_msgs::Point& center,
        const std::vector<geometry_msgs::Point>& accessible_area,
        const std::vector<geometry_msgs::Point>& polygon);
    std::vector<geometry_msgs::PoseStamped> generatePerimeterCoverage(
        const std::vector<geometry_msgs::Point>& obstacle_contour,
        const std::vector<geometry_msgs::Point>& polygon);
    
    // Advanced pathfinding
    std::vector<geometry_msgs::Point> findPathAroundObstacle(
        const geometry_msgs::Point& start, 
        const geometry_msgs::Point& goal,
        const std::vector<geometry_msgs::Point>& polygon);
    
    std::vector<geometry_msgs::Point> aStarPathfinding(
        const geometry_msgs::Point& start,
        const geometry_msgs::Point& goal,
        const std::vector<geometry_msgs::Point>& polygon);
    
    // Obstacle analysis and handling
    std::vector<geometry_msgs::Point> getObstacleContour(double center_x, double center_y, double search_radius);
    std::vector<geometry_msgs::Point> getDetailedObstacleContour(double center_x, double center_y, double search_radius);
    bool findAlternativeRoute(const geometry_msgs::Point& start, 
                             const geometry_msgs::Point& blocked_point,
                             const geometry_msgs::Point& target,
                             std::vector<geometry_msgs::Point>& alternative_path);
    
    // Area analysis functions
    bool isAreaAccessible(const std::vector<geometry_msgs::Point>& area_points, double min_width);
    double calculateAreaSize(const std::vector<geometry_msgs::Point>& area_points);
    std::vector<std::vector<geometry_msgs::Point>> subdivideArea(
        const std::vector<geometry_msgs::Point>& area_points);
    
    // Integration functions
    void integrateObstacleCoverageIntoPlan(
        std::vector<geometry_msgs::PoseStamped>& main_plan,
        const std::vector<geometry_msgs::PoseStamped>& obstacle_coverage,
        const geometry_msgs::Point& connection_point);
    
    // Dynamic spacing adjustment
    double calculateAdaptiveSpacing(double y_position, const std::vector<geometry_msgs::Point>& polygon);
    bool hasNearbyObstacles(double x, double y, double search_radius);
    double getObstacleDensity(double center_x, double center_y, double search_radius);
    
    // Path optimization and validation
    std::vector<geometry_msgs::PoseStamped> smoothPath(const std::vector<geometry_msgs::PoseStamped>& raw_path);
    bool isLineOfSightClear(const geometry_msgs::Point& start, const geometry_msgs::Point& end);
    bool validatePath(const std::vector<geometry_msgs::PoseStamped>& path);
    std::vector<geometry_msgs::PoseStamped> refinePath(const std::vector<geometry_msgs::PoseStamped>& path);
    bool findClearPathSegment(const geometry_msgs::Point& start_p, const geometry_msgs::Point& end_p,
                              std::vector<geometry_msgs::PoseStamped>& segment_plan, double orientation_yaw,
                              const std::vector<geometry_msgs::Point>& current_polygon_geom);
    
    // Connection between segments
    std::vector<geometry_msgs::PoseStamped> createConnectionPath(
        const geometry_msgs::Point& from,
        const geometry_msgs::Point& to,
        double orientation,
        const std::vector<geometry_msgs::Point>& polygon);
    
    // Utility functions
    double euclideanDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b);
    double manhattanDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b);
    geometry_msgs::Point worldToGrid(double world_x, double world_y);
    geometry_msgs::Point gridToWorld(double grid_x, double grid_y);
    std::string pointToString(double x, double y);
    geometry_msgs::PoseStamped createPoseStamped(const geometry_msgs::Point& point, double yaw);
    geometry_msgs::Point createPoint(double x, double y, double z = 0.0);
    
    // Coverage completeness verification
    bool verifyCoverageCompleteness(const std::vector<geometry_msgs::PoseStamped>& plan,
                                   const std::vector<geometry_msgs::Point>& polygon);
    std::vector<geometry_msgs::Point> findUncoveredAreas(const std::vector<geometry_msgs::PoseStamped>& plan,
                                                         const std::vector<geometry_msgs::Point>& polygon);
};

} // namespace blm6191_coverage_planners

#endif // FULL_COVERAGE_PLANNER_H