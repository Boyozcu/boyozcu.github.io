#ifndef PURE_PURSUIT_LOCAL_PLANNER_H
#define PURE_PURSUIT_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>
#include <base_local_planner/odometry_helper_ros.h> // OdometryHelperRos için


namespace blm6191_coverage_planners {

class PurePursuitLocalPlanner : public nav_core::BaseLocalPlanner {
public:
    PurePursuitLocalPlanner();
    PurePursuitLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros); // Bu constructor genellikle pluginlib tarafından kullanılmaz
    ~PurePursuitLocalPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;

private:
    geometry_msgs::PoseStamped findLookaheadPoint(const geometry_msgs::PoseStamped& robot_pose_in_global_frame);
    void publishCarrotMarker(const geometry_msgs::Point& carrot_point_global);

    bool initialized_;
    tf2_ros::Buffer* tf_buffer_; // Pointer olarak saklanacak
    costmap_2d::Costmap2DROS* costmap_ros_; // Pointer olarak saklanacak
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    
    double lookahead_distance_;
    double linear_vel_;
    double max_angular_vel_;
    double goal_dist_tolerance_;
    std::string robot_base_frame_;
    std::string global_frame_; 

    size_t current_segment_idx_; 
    bool goal_reached_;

    ros::Publisher carrot_pub_; 
    base_local_planner::OdometryHelperRos odom_helper_; // Odometri verilerini almak için (opsiyonel)

};

} // namespace blm6191_coverage_planners

#endif // PURE_PURSUIT_LOCAL_PLANNER_H
