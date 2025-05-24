#include "full_coverage_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <algorithm>
#include <vector>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(blm6191_coverage_planners::FullCoveragePlanner, nav_core::BaseGlobalPlanner)

namespace blm6191_coverage_planners {

FullCoveragePlanner::FullCoveragePlanner() : initialized_(false), costmap_ros_(NULL), costmap_(NULL) {}

FullCoveragePlanner::FullCoveragePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : initialized_(false), costmap_ros_(NULL), costmap_(NULL) {
    initialize(name, costmap_ros);
}

void FullCoveragePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ros_ = costmap_ros;
        if(costmap_ros_){
            costmap_ = costmap_ros_->getCostmap();
        } else {
            ROS_ERROR("[%s] Costmap ROS Wrapper is null in initialize. Cannot get costmap.", name.c_str());
            return;
        }
        if (!costmap_) {
             ROS_ERROR("[%s] Failed to get costmap from ROS Wrapper in initialize.", name.c_str());
             return;
        }

        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("robot_radius", robot_radius_, 0.17);
        private_nh.param("sweep_spacing_factor", sweep_spacing_factor_, 0.9);
        private_nh.param("path_point_distance", path_point_distance_, 0.1);
        private_nh.param("frame_id", frame_id_, std::string("map"));
        private_nh.param("use_param_poly", use_param_poly_, true);

        if (use_param_poly_) {
            private_nh.param("p1_x", p1x_, 0.0); private_nh.param("p1_y", p1y_, 0.0);
            private_nh.param("p2_x", p2x_, 0.0); private_nh.param("p2_y", p2y_, 0.0);
            private_nh.param("p3_x", p3x_, 0.0); private_nh.param("p3_y", p3y_, 0.0);
            private_nh.param("p4_x", p4x_, 0.0); private_nh.param("p4_y", p4y_, 0.0);
            
            polygon_corners_msg_.resize(4);
            polygon_corners_msg_[0].x = p1x_; polygon_corners_msg_[0].y = p1y_;
            polygon_corners_msg_[1].x = p2x_; polygon_corners_msg_[1].y = p2y_;
            polygon_corners_msg_[2].x = p3x_; polygon_corners_msg_[2].y = p3y_;
            polygon_corners_msg_[3].x = p4x_; polygon_corners_msg_[3].y = p4y_;
        } else {
            ros::NodeHandle nh;
            polygon_sub_ = nh.subscribe("/coverage_polygon", 1, &FullCoveragePlanner::polygonCallback, this);
        }
        
        plan_pub_ = private_nh.advertise<nav_msgs::Path>("coverage_plan", 1);
        initialized_ = true;
        ROS_INFO("[%s] FullCoveragePlanner initialized.", name.c_str());
    }
}

void FullCoveragePlanner::polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg) {
    if (!use_param_poly_) {
        if (msg->polygon.points.size() == 4) {
            polygon_corners_msg_ = msg->polygon.points;
            ROS_INFO("FullCoveragePlanner: Received polygon with 4 corners via topic.");
        } else {
            ROS_WARN("FullCoveragePlanner: Received polygon via topic but it does not have 4 corners (%zu). Ignoring.", msg->polygon.points.size());
            polygon_corners_msg_.clear();
        }
    }
}

bool FullCoveragePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_ || polygon_corners_msg_.size() != 4 || !costmap_) {
        ROS_ERROR_COND(!initialized_, "[FullCoveragePlanner] Not initialized.");
        ROS_ERROR_COND(polygon_corners_msg_.size() != 4, "[FullCoveragePlanner] Polygon not defined (4 corners needed).");
        ROS_ERROR_COND(!costmap_, "[FullCoveragePlanner] Costmap not available.");
        return false;
    }

    plan.clear();
    std::vector<geometry_msgs::Point> poly_corners_geom(polygon_corners_msg_.size());
    for(size_t i=0; i < polygon_corners_msg_.size(); ++i) {
        poly_corners_geom[i].x = polygon_corners_msg_[i].x;
        poly_corners_geom[i].y = polygon_corners_msg_[i].y;
        poly_corners_geom[i].z = 0.0;
    }

    generateBoustrophedonPath(poly_corners_geom, start, plan);

    if (!plan.empty()) {
        nav_msgs::Path gui_path;
        gui_path.poses.resize(plan.size());
        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();
        for (unsigned int i = 0; i < plan.size(); i++) {
            plan[i].header.frame_id = frame_id_;
            plan[i].header.stamp = ros::Time::now();
            gui_path.poses[i] = plan[i];
        }
        plan_pub_.publish(gui_path);
        ROS_INFO("[FullCoveragePlanner] Plan generated with %zu points.", plan.size());
        return true;
    } else {
        ROS_WARN("[FullCoveragePlanner] Generated plan is empty.");
        return false;
    }
}

bool FullCoveragePlanner::isPointInPolygon(const geometry_msgs::Point& p, const std::vector<geometry_msgs::Point>& poly_points_geom) {
    int i, j;
    bool c = false;
    int nvert = poly_points_geom.size();
    if (nvert < 3) return false;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((poly_points_geom[i].y > p.y) != (poly_points_geom[j].y > p.y)) &&
            (poly_points_geom[j].y != poly_points_geom[i].y) && 
            (p.x < (poly_points_geom[j].x - poly_points_geom[i].x) * (p.y - poly_points_geom[i].y) / (poly_points_geom[j].y - poly_points_geom[i].y) + poly_points_geom[i].x)) {
            c = !c;
        }
    }
    return c;
}

bool FullCoveragePlanner::isObstacle(double world_x, double world_y) {
    if (!costmap_) return true;
    unsigned int map_x, map_y;
    if (!costmap_->worldToMap(world_x, world_y, map_x, map_y)) {
        return true;
    }
    unsigned char cost = costmap_->getCost(map_x, map_y);
    return (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost != costmap_2d::NO_INFORMATION);
}

bool FullCoveragePlanner::findClearPathSegment(const geometry_msgs::Point& start_p, const geometry_msgs::Point& end_p,
                                               std::vector<geometry_msgs::PoseStamped>& segment_plan, double orientation_yaw,
                                               const std::vector<geometry_msgs::Point>& current_polygon_geom) {
    segment_plan.clear();
    double dx = end_p.x - start_p.x;
    double dy = end_p.y - start_p.y;
    double distance = std::hypot(dx, dy);
    double effective_ppd = std::max(path_point_distance_ * 0.5, costmap_ ? costmap_->getResolution() : 0.01);
    if (effective_ppd < 0.01) effective_ppd = 0.01;

    if (distance < effective_ppd / 2.0) {
        if(!isObstacle(end_p.x, end_p.y) && isPointInPolygon(end_p, current_polygon_geom)) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position = end_p;
            pose.pose.position.z = 0.01;
            tf2::Quaternion q_tf; q_tf.setRPY(0,0,orientation_yaw);
            pose.pose.orientation = tf2::toMsg(q_tf);
            segment_plan.push_back(pose);
            return true;
        }
        return false;
    }

    int num_steps = static_cast<int>(distance / effective_ppd);
    if (num_steps <= 0) num_steps = 1;

    for (int i = 0; i <= num_steps; ++i) {
        double t = static_cast<double>(i) / num_steps;
        geometry_msgs::Point current_p;
        current_p.x = start_p.x + t * dx;
        current_p.y = start_p.y + t * dy;
        current_p.z = 0.0;
        
        if (isObstacle(current_p.x, current_p.y) || !isPointInPolygon(current_p, current_polygon_geom)) {
            segment_plan.clear();
            return false;
        }
        geometry_msgs::PoseStamped pose;
        pose.pose.position = current_p;
        pose.pose.position.z = 0.01;
        tf2::Quaternion q_tf;
        q_tf.setRPY(0, 0, orientation_yaw);
        pose.pose.orientation = tf2::toMsg(q_tf);
        segment_plan.push_back(pose);
    }
    return true;
}

void FullCoveragePlanner::generateBoustrophedonPath(
    const std::vector<geometry_msgs::Point>& poly_corners_geom,
    const geometry_msgs::PoseStamped& start_pose_unused,
    std::vector<geometry_msgs::PoseStamped>& plan) {

    if (poly_corners_geom.size() != 4) { ROS_ERROR("[FullCoveragePlanner::generateBoustrophedonPath] Polygon != 4 corners."); return; }
    if (robot_radius_ <= 0 || sweep_spacing_factor_ <= 0 || path_point_distance_ <= 0) { ROS_ERROR("[FullCoveragePlanner::generateBoustrophedonPath] Invalid params"); return; }
    if (!costmap_) { ROS_ERROR("[FullCoveragePlanner::generateBoustrophedonPath] Costmap unavailable!"); return; }
    
    double min_x_poly = poly_corners_geom[0].x, max_x_poly = poly_corners_geom[0].x;
    double min_y_poly = poly_corners_geom[0].y, max_y_poly = poly_corners_geom[0].y;
    for (size_t i = 1; i < poly_corners_geom.size(); ++i) {
        min_x_poly = std::min(min_x_poly, poly_corners_geom[i].x);
        max_x_poly = std::max(max_x_poly, poly_corners_geom[i].x);
        min_y_poly = std::min(min_y_poly, poly_corners_geom[i].y);
        max_y_poly = std::max(max_y_poly, poly_corners_geom[i].y);
    }

    double line_separation = (2.0 * robot_radius_) * sweep_spacing_factor_;
    if (line_separation < costmap_->getResolution() * 1.5 ){ 
        line_separation = costmap_->getResolution() * 1.5; 
        ROS_WARN("[FullCoveragePlanner] Sweep separation adjusted to %.2f due to map resolution.", line_separation);
    }
    double effective_path_pt_dist = std::max(path_point_distance_, costmap_->getResolution());

    bool current_sweep_left_to_right = true; // Düzeltilmiş değişken adı
    double yaw_l_to_r = 0.0;
    double yaw_r_to_l = M_PI;
    
    geometry_msgs::PoseStamped last_valid_pose_on_plan; // Tanımlandı
    bool plan_already_has_points = false; // Tanımlandı ve adı düzeltildi

    for (double y_sweep_center = min_y_poly + robot_radius_; 
         y_sweep_center <= max_y_poly - robot_radius_; 
         y_sweep_center += line_separation) {
        
        std::vector<double> x_intersections_on_line;
        for (size_t i = 0; i < poly_corners_geom.size(); ++i) {
            const auto& p1 = poly_corners_geom[i];
            const auto& p2 = poly_corners_geom[(i + 1) % poly_corners_geom.size()];
            if (std::abs(p1.y - p2.y) > 1e-6) { 
                if ((y_sweep_center >= std::min(p1.y, p2.y) - 1e-6) && (y_sweep_center <= std::max(p1.y, p2.y) + 1e-6)) {
                    double intersect_x = p1.x + (p2.x - p1.x) * (y_sweep_center - p1.y) / (p2.y - p1.y);
                    if (intersect_x >= min_x_poly - robot_radius_ && intersect_x <= max_x_poly + robot_radius_) {
                         x_intersections_on_line.push_back(intersect_x);
                    }
                }
            }
        }
        std::sort(x_intersections_on_line.begin(), x_intersections_on_line.end());
        x_intersections_on_line.erase(std::unique(x_intersections_on_line.begin(), x_intersections_on_line.end(), 
                                     [](double a, double b){ return std::abs(a-b) < 1e-5; }), x_intersections_on_line.end());

        double current_sweep_yaw = current_sweep_left_to_right ? yaw_l_to_r : yaw_r_to_l; // Düzeltilmiş değişken adı

        for (size_t k = 0; k + 1 < x_intersections_on_line.size(); k += 2) {
            double x_raw_start = x_intersections_on_line[k];
            double x_raw_end = x_intersections_on_line[k+1];

            geometry_msgs::Point mid_p_seg_test; 
            mid_p_seg_test.x = (x_raw_start + x_raw_end) / 2.0; 
            mid_p_seg_test.y = y_sweep_center;
            if (!isPointInPolygon(mid_p_seg_test, poly_corners_geom)) {
                continue; 
            }
            
            double x_eff_start = x_raw_start + robot_radius_;
            double x_eff_end   = x_raw_end   - robot_radius_;

            if (x_eff_start >= x_eff_end - effective_path_pt_dist / 2.0) continue;

            double x_iter, x_limit_iter, x_step_iter;
            if (current_sweep_left_to_right) { // Düzeltilmiş değişken adı
                x_iter = x_eff_start; x_limit_iter = x_eff_end; x_step_iter = effective_path_pt_dist;
            } else {
                x_iter = x_eff_end; x_limit_iter = x_eff_start; x_step_iter = -effective_path_pt_dist;
            }

            std::vector<geometry_msgs::PoseStamped> current_sub_segment_plan;
            bool obstructed_on_this_sub_segment = false;

            while ((current_sweep_left_to_right ? x_iter <= x_limit_iter + 1e-3 : x_iter >= x_limit_iter - 1e-3)) { // Düzeltilmiş değişken adı
                geometry_msgs::Point current_point_candidate;
                current_point_candidate.x = x_iter;
                current_point_candidate.y = y_sweep_center;
                current_point_candidate.z = 0.0;

                if (isPointInPolygon(current_point_candidate, poly_corners_geom) && !isObstacle(x_iter, y_sweep_center)) {
                    if (obstructed_on_this_sub_segment) { 
                        if (!current_sub_segment_plan.empty()) {
                             if (plan_already_has_points) { // Düzeltilmiş değişken adı
                                std::vector<geometry_msgs::PoseStamped> turning_path_nodes;
                                tf2::Quaternion q_prev_orient_tf; tf2::fromMsg(last_valid_pose_on_plan.pose.orientation, q_prev_orient_tf);
                                double r,p,yaw_prev; tf2::Matrix3x3(q_prev_orient_tf).getRPY(r,p,yaw_prev);
                                if (findClearPathSegment(last_valid_pose_on_plan.pose.position, current_sub_segment_plan.front().pose.position, turning_path_nodes, yaw_prev, poly_corners_geom)) {
                                    plan.insert(plan.end(), turning_path_nodes.begin(), turning_path_nodes.end());
                                }
                            }
                            plan.insert(plan.end(), current_sub_segment_plan.begin(), current_sub_segment_plan.end());
                            if (!plan.empty()) {last_valid_pose_on_plan = plan.back(); plan_already_has_points = true;} // Düzeltilmiş değişken adı
                            current_sub_segment_plan.clear();
                        }
                        obstructed_on_this_sub_segment = false; 
                    }

                    geometry_msgs::PoseStamped new_pose;
                    new_pose.pose.position = current_point_candidate;
                    new_pose.pose.position.z = 0.01;
                    tf2::Quaternion q_tf;
                    q_tf.setRPY(0, 0, current_sweep_yaw);
                    new_pose.pose.orientation = tf2::toMsg(q_tf);
                    current_sub_segment_plan.push_back(new_pose);
                } else { 
                    obstructed_on_this_sub_segment = true;
                    if (!current_sub_segment_plan.empty()) { 
                         if (plan_already_has_points) { // Düzeltilmiş değişken adı
                            std::vector<geometry_msgs::PoseStamped> turning_path_nodes;
                            tf2::Quaternion q_prev_orient_tf; tf2::fromMsg(last_valid_pose_on_plan.pose.orientation, q_prev_orient_tf);
                            double r,p,yaw_prev; tf2::Matrix3x3(q_prev_orient_tf).getRPY(r,p,yaw_prev);
                            if (findClearPathSegment(last_valid_pose_on_plan.pose.position, current_sub_segment_plan.front().pose.position, turning_path_nodes, yaw_prev, poly_corners_geom)) {
                                plan.insert(plan.end(), turning_path_nodes.begin(), turning_path_nodes.end());
                            }
                        }
                        plan.insert(plan.end(), current_sub_segment_plan.begin(), current_sub_segment_plan.end());
                        if (!plan.empty()) { last_valid_pose_on_plan = plan.back(); plan_already_has_points = true;} // Düzeltilmiş değişken adı
                        current_sub_segment_plan.clear();
                    }
                    if(current_sweep_left_to_right) x_iter += robot_radius_ * 1.5; else x_iter -= robot_radius_ * 1.5; // Düzeltilmiş değişken adı
                    continue;
                }
                x_iter += x_step_iter;
            }
            if (!current_sub_segment_plan.empty()) {
                if (plan_already_has_points) { // Düzeltilmiş değişken adı
                    std::vector<geometry_msgs::PoseStamped> turning_path_nodes;
                    tf2::Quaternion q_prev_orient_tf; tf2::fromMsg(last_valid_pose_on_plan.pose.orientation, q_prev_orient_tf);
                    double r,p,yaw_prev; tf2::Matrix3x3(q_prev_orient_tf).getRPY(r,p,yaw_prev);
                    if (findClearPathSegment(last_valid_pose_on_plan.pose.position, current_sub_segment_plan.front().pose.position, turning_path_nodes, yaw_prev, poly_corners_geom)) {
                        plan.insert(plan.end(), turning_path_nodes.begin(), turning_path_nodes.end());
                    }
                }
                plan.insert(plan.end(), current_sub_segment_plan.begin(), current_sub_segment_plan.end());
                 if (!plan.empty()) { last_valid_pose_on_plan = plan.back(); plan_already_has_points = true;} // Düzeltilmiş değişken adı
            }
        }
        current_sweep_left_to_right = !current_sweep_left_to_right; // Düzeltilmiş değişken adı
        // Bir sonraki süpürme hattı için ana yönelim current_sweep_yaw döngünün başında ayarlanacak
    }

    if (plan.empty()) {
        ROS_WARN("[FullCoveragePlanner::generateBoustrophedonPath] No valid points generated.");
    } else {
        ROS_INFO("[FullCoveragePlanner::generateBoustrophedonPath] Path generated with %zu points.", plan.size());
    }
}

} // namespace blm6191_coverage_planners
