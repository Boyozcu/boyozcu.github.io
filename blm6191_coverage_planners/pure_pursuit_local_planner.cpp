#include "pure_pursuit_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm> // std::min, std::max

PLUGINLIB_EXPORT_CLASS(blm6191_coverage_planners::PurePursuitLocalPlanner, nav_core::BaseLocalPlanner)

namespace blm6191_coverage_planners {

// Constructor, initialize, setPlan, computeVelocityCommands (robot_pose transform kısmı hariç), isGoalReached, publishCarrotMarker
// bir önceki "tümünü gönder" mesajındaki gibi kalabilir. Ana değişiklik findLookaheadPoint ve onu çağıran yerlerde.
// Sadece initialize ve findLookaheadPoint'i ve computeVelocityCommands'in başını güncelleyelim.

PurePursuitLocalPlanner::PurePursuitLocalPlanner() : initialized_(false), goal_reached_(false), tf_buffer_(NULL), costmap_ros_(NULL), odom_helper_("odom") {}

PurePursuitLocalPlanner::~PurePursuitLocalPlanner() {}

void PurePursuitLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        tf_buffer_ = tf;
        costmap_ros_ = costmap_ros;
        if (!costmap_ros_ || !tf_buffer_) {
            ROS_FATAL("[%s] Costmap ROS Wrapper or TF Buffer is NULL in initialize!", name.c_str());
            return;
        }

        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("lookahead_distance", lookahead_distance_, 0.9);
        private_nh.param("linear_vel", linear_vel_, 0.10);
        private_nh.param("max_angular_vel", max_angular_vel_, 0.55);
        private_nh.param("goal_dist_tolerance", goal_dist_tolerance_, 0.20);
        private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_footprint"));
        
        global_frame_ = "map"; 

        odom_helper_.setOdomTopic(ros::NodeHandle().resolveName("odom"));
        carrot_pub_ = private_nh.advertise<visualization_msgs::Marker>("carrot_marker", 1, true);
        
        initialized_ = true;
        ROS_INFO("[%s] PurePursuitLocalPlanner initialized. Lookahead: %.2f, LinVel: %.2f, MaxAngVel: %.2f, GoalTol: %.2f, BaseFrame: %s, GlobalFrame: %s",
                 name.c_str(), lookahead_distance_, linear_vel_, max_angular_vel_, goal_dist_tolerance_, robot_base_frame_.c_str(), global_frame_.c_str());
    }
}

bool PurePursuitLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (!initialized_) { ROS_ERROR_THROTTLE(1.0,"PurePursuitLocalPlanner: Not initialized."); return false; }
    
    goal_reached_ = false;
    current_segment_idx_ = 0; // Her yeni planda indeksi sıfırla

    if (orig_global_plan.empty()) {
        ROS_WARN("PurePursuitLocalPlanner: Received an empty plan. Marking goal as reached.");
        global_plan_.clear();
        goal_reached_ = true;
        return true; // Boş plan geçerli bir durum olabilir (hedefe çoktan ulaşıldı vs.)
    }

    global_plan_ = orig_global_plan;

    // Planın geçerli bir frame_id'si olduğundan ve global_frame_ ile eşleştiğinden emin ol
    if (global_plan_.front().header.frame_id != global_frame_) {
        ROS_ERROR("PurePursuitLocalPlanner: Plan frame ('%s') MISMATCHES expected global frame ('%s')! This will cause TF errors. Clearing plan.",
                 global_plan_.front().header.frame_id.c_str(), global_frame_.c_str());
        global_plan_.clear();
        goal_reached_ = true;
        return false; // Hatalı çerçeveyle planı kabul etme
    }

    ROS_INFO("PurePursuitLocalPlanner: Received new plan with %zu points in frame '%s'.",
             global_plan_.size(), global_frame_.c_str());
    return true;
}


// computeVelocityCommands fonksiyonunun sadece başı, robot pozu alma ve TF kısmı
bool PurePursuitLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

    if (!initialized_ || global_plan_.empty() || goal_reached_) {
        // Durum logları setPlan ve isGoalReached içinde daha iyi ele alınabilir
        return goal_reached_ || global_plan_.empty(); // Eğer hedefe ulaşıldıysa veya plan yoksa başarılı say
    }

    geometry_msgs::PoseStamped robot_pose_from_costmap;
    if (!costmap_ros_->getRobotPose(robot_pose_from_costmap)) {
        ROS_ERROR_THROTTLE(1.0, "PurePursuitLocalPlanner: Failed to get robot pose from costmap.");
        return false;
    }

    geometry_msgs::PoseStamped robot_pose_in_global_frame = robot_pose_from_costmap;
    if (robot_pose_from_costmap.header.frame_id != global_frame_) {
        try {
            geometry_msgs::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
                global_frame_, robot_pose_from_costmap.header.frame_id, ros::Time(0), ros::Duration(0.2));
            tf2::doTransform(robot_pose_from_costmap, robot_pose_in_global_frame, tf_stamped);
            robot_pose_in_global_frame.header.frame_id = global_frame_;
        } catch (tf2::TransformException &ex) {
            ROS_ERROR_THROTTLE(1.0, "PurePursuitLocalPlanner: TF Exception transforming robot pose to '%s': %s", global_frame_.c_str(), ex.what());
            return false;
        }
    }

    // Hedefin son noktasına ulaşıldı mı kontrol et (bu, isGoalReached'in de temeli olabilir)
    const geometry_msgs::PoseStamped& final_goal_on_plan = global_plan_.back();
    double dist_to_final_goal = std::hypot(final_goal_on_plan.pose.position.x - robot_pose_in_global_frame.pose.position.x,
                                           final_goal_on_plan.pose.position.y - robot_pose_in_global_frame.pose.position.y);

    if (dist_to_final_goal < goal_dist_tolerance_) {
        ROS_INFO_ONCE("PurePursuitLocalPlanner: Goal reached! Distance to final plan point: %.3f m", dist_to_final_goal);
        goal_reached_ = true;
        return true; // Hız komutları zaten sıfır, hedefteyiz.
    }
    
    // ---- findLookaheadPoint çağrısı ve sonrası önceki mesajdaki gibi devam eder ----
    // Sadece findLookaheadPoint fonksiyonunu aşağıdakilerle değiştireceğiz.
    // ... (computeVelocityCommands'in geri kalanı önceki mesajdaki gibi) ...
    geometry_msgs::PoseStamped lookahead_point_in_global_frame = findLookaheadPoint(robot_pose_in_global_frame);

    if (lookahead_point_in_global_frame.header.frame_id.empty()) {
        ROS_WARN_THROTTLE(1.0, "PurePursuitLocalPlanner: No valid lookahead point found. Stopping.");
        return false; 
    }
    
    publishCarrotMarker(lookahead_point_in_global_frame.pose.position);

    geometry_msgs::PoseStamped lookahead_point_in_robot_frame;
    try {
        geometry_msgs::TransformStamped tf_global_to_robot = tf_buffer_->lookupTransform(
            robot_base_frame_, global_frame_, ros::Time(0), ros::Duration(0.1));
        tf2::doTransform(lookahead_point_in_global_frame, lookahead_point_in_robot_frame, tf_global_to_robot);
    } catch (tf2::TransformException& ex) {
        ROS_ERROR_THROTTLE(1.0, "PurePursuitLocalPlanner: TF Exception transforming lookahead point: %s", ex.what());
        return false;
    }

    double x_local_carrot = lookahead_point_in_robot_frame.pose.position.x;
    double y_local_carrot = lookahead_point_in_robot_frame.pose.position.y;
    
    double alpha_to_carrot = atan2(y_local_carrot, x_local_carrot);
    double actual_dist_to_carrot = std::hypot(x_local_carrot, y_local_carrot);
    
    if (actual_dist_to_carrot < 0.01) {
        cmd_vel.linear.x = linear_vel_ * 0.3;
        cmd_vel.angular.z = 0.0;
        return true;
    }

    double curvature = (2.0 * sin(alpha_to_carrot)) / actual_dist_to_carrot;
    double target_angular_velocity = linear_vel_ * curvature;

    cmd_vel.linear.x = linear_vel_;
    cmd_vel.angular.z = std::max(-max_angular_vel_, std::min(max_angular_vel_, target_angular_velocity));
    
    ROS_DEBUG_THROTTLE(0.2, "PurePursuit: CmdLin=%.2f, CmdAng=%.2f (rawAng: %.2f), Alpha=%.2f rad, CarrotDist=%.2f m",
              cmd_vel.linear.x, cmd_vel.angular.z, target_angular_velocity, alpha_to_carrot, actual_dist_to_carrot);
    return true;
}


geometry_msgs::PoseStamped PurePursuitLocalPlanner::findLookaheadPoint(const geometry_msgs::PoseStamped& robot_pose_global) {
    geometry_msgs::PoseStamped lookahead_pt;
    lookahead_pt.header.frame_id = ""; // Başlangıçta geçersiz

    if (global_plan_.empty()) {
        ROS_WARN_THROTTLE(1.0, "[PurePursuit::findLookaheadPoint] Global plan is empty.");
        return lookahead_pt;
    }
    lookahead_pt.header.frame_id = global_frame_; // Hedef noktamız her zaman global_frame_'de olacak

    // 1. Robota en yakın yol segmentini/noktasını bul
    //    Bu, robot yoldan saptığında veya yol karmaşık olduğunda önemlidir.
    //    current_segment_idx_'ı bu en yakın segmente göre güncelleyebiliriz.
    double min_dist_sq_to_path = std::numeric_limits<double>::max();
    size_t closest_segment_start_idx = current_segment_idx_; // Aramaya mevcut yerden başla, ama gerekirse geriye de bak

    // current_segment_idx_'ı çok fazla geriye sarmamak için bir sınır (örn: son 10 segment)
    size_t search_start_idx = (current_segment_idx_ > 10) ? (current_segment_idx_ - 10) : 0;

    for (size_t i = search_start_idx; i < global_plan_.size(); ++i) {
        double dist_sq = pow(global_plan_[i].pose.position.x - robot_pose_global.pose.position.x, 2) +
                         pow(global_plan_[i].pose.position.y - robot_pose_global.pose.position.y, 2);
        if (dist_sq < min_dist_sq_to_path) {
            min_dist_sq_to_path = dist_sq;
            closest_segment_start_idx = i;
        }
    }
    current_segment_idx_ = closest_segment_start_idx; // En yakın noktadan aramaya başla/devam et
    ROS_DEBUG("[PurePursuit::findLookaheadPoint] Closest point on plan (index %zu) to robot.", current_segment_idx_);


    // 2. En yakın segmentten başlayarak lookahead mesafesindeki kesişim noktasını ara
    for (size_t i = current_segment_idx_; i < global_plan_.size(); ++i) {
        // Son noktaya ulaşıldıysa ve lookahead mesafesi içindeyse, son noktayı hedefle
        if (i == global_plan_.size() - 1) {
            double dist_sq_to_last = pow(global_plan_[i].pose.position.x - robot_pose_global.pose.position.x, 2) +
                                     pow(global_plan_[i].pose.position.y - robot_pose_global.pose.position.y, 2);
            if (dist_sq_to_last <= pow(lookahead_distance_, 2)) {
                lookahead_pt = global_plan_[i];
                current_segment_idx_ = i; // Son nokta hedefleniyorsa index'i güncelle
                ROS_DEBUG("[PurePursuit::findLookaheadPoint] Path end reached, targeting final point within lookahead distance.");
                return lookahead_pt;
            }
            // Son nokta lookahead dışında ise, bir önceki segmentte kesişim bulunmalıydı
            break; 
        }

        const geometry_msgs::Point& p1 = global_plan_[i].pose.position;
        const geometry_msgs::Point& p2 = global_plan_[i+1].pose.position;
        const geometry_msgs::Point& robot_p = robot_pose_global.pose.position;

        double dx_seg = p2.x - p1.x;
        double dy_seg = p2.y - p1.y;
        double drx = p1.x - robot_p.x; // p1 - robot
        double dry = p1.y - robot_p.y; // p1 - robot

        double a = dx_seg * dx_seg + dy_seg * dy_seg;
        double b = 2 * (drx * dx_seg + dry * dy_seg);
        double c = drx * drx + dry * dry - lookahead_distance_ * lookahead_distance_;
        double discriminant = b * b - 4 * a * c;

        if (discriminant >= 0) {
            if (std::abs(a) < 1e-9) { // Segment çok kısa veya p1=p2, sadece p1'i kontrol et
                 if (c <= 0) { // robot p1'e Ld içinde
                     lookahead_pt = global_plan_[i];
                     current_segment_idx_ = i;
                     return lookahead_pt;
                 }
                continue;
            }
            discriminant = sqrt(discriminant);
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);
            
            // İlerideki ve segment üzerindeki kesişimi seç (0 <= t <= 1)
            if (t2 >= 0 && t2 <= 1.0) { // t2 genellikle daha ilerideki çözümdür
                lookahead_pt.pose.position.x = p1.x + t2 * dx_seg;
                lookahead_pt.pose.position.y = p1.y + t2 * dy_seg;
                lookahead_pt.pose.position.z = p1.z;
                lookahead_pt.pose.orientation = global_plan_[i+1].pose.orientation; // Segment sonunun yönelimi
                current_segment_idx_ = i; // Bu segment üzerinde kesişim bulundu
                ROS_DEBUG("[PurePursuit::findLookaheadPoint] Found lookahead on segment %zu (t=%.2f)", i, t2);
                return lookahead_pt;
            } else if (t1 >= 0 && t1 <= 1.0) {
                lookahead_pt.pose.position.x = p1.x + t1 * dx_seg;
                lookahead_pt.pose.position.y = p1.y + t1 * dy_seg;
                lookahead_pt.pose.position.z = p1.z;
                lookahead_pt.pose.orientation = global_plan_[i+1].pose.orientation;
                current_segment_idx_ = i;
                ROS_DEBUG("[PurePursuit::findLookaheadPoint] Found lookahead on segment %zu (t=%.2f)", i, t1);
                return lookahead_pt;
            }
        }
        // Bu segmentte kesişim yoksa veya discriminant < 0 ise, sonraki segmente bak.
        // Robot bu segmenti geçmiş olabilir.
    }
    
    // Eğer hiçbir segmentte kesişim bulunamadıysa (örn: robot yoldan çok saptı
    // veya yol lookahead_distance'dan daha kısa ve son nokta da dışarıda kaldı),
    // en mantıklısı yolun SON NOKTASINI hedeflemektir.
    if (!global_plan_.empty()) {
        ROS_WARN_THROTTLE(1.0, "[PurePursuit::findLookaheadPoint] No valid intersection on path segments. Defaulting to last point of the plan.");
        lookahead_pt = global_plan_.back();
        current_segment_idx_ = global_plan_.size() - 1; // İlerlemeyi en sona al
        return lookahead_pt;
    }
    
    ROS_ERROR_THROTTLE(1.0, "[PurePursuit::findLookaheadPoint] Critical error: Could not find any lookahead point and plan is empty.");
    lookahead_pt.header.frame_id = ""; // Geçersiz olarak işaretle
    return lookahead_pt;
}

bool PurePursuitLocalPlanner::isGoalReached() {
    if (!initialized_) return false; // Henüz başlamadıysa hedefe ulaşılmamıştır
    // computeVelocityCommands içinde goal_reached_ bayrağı ayarlanıyor.
    return goal_reached_;
}

void PurePursuitLocalPlanner::publishCarrotMarker(const geometry_msgs::Point& carrot_point_in_global_frame) {
    if (carrot_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::Marker marker;
    marker.header.frame_id = global_frame_; // Carrot her zaman global_frame_'de
    marker.header.stamp = ros::Time::now();
    marker.ns = "pure_pursuit_carrot";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position = carrot_point_in_global_frame;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.25; marker.scale.y = 0.25; marker.scale.z = 0.25; // Biraz büyütüldü
    marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; marker.color.a = 0.9; 
    marker.lifetime = ros::Duration(0.5); // Yarım saniye görünsün
    
    carrot_pub_.publish(marker);
}

} // namespace blm6191_coverage_planners
