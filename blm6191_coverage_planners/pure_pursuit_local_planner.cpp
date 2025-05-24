#include "pure_pursuit_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h> // normalize_angle_positive, shortest_angular_distance
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // tf2::doTransform
#include <tf2/utils.h> // tf2::getYaw (ancak Matrix3x3 ile RPY almayı tercih edebiliriz)
#include <tf2/LinearMath/Matrix3x3.h> // getRPY için
#include <tf2/LinearMath/Quaternion.h>


PLUGINLIB_EXPORT_CLASS(blm6191_coverage_planners::PurePursuitLocalPlanner, nav_core::BaseLocalPlanner)

namespace blm6191_coverage_planners {

PurePursuitLocalPlanner::PurePursuitLocalPlanner() : initialized_(false), goal_reached_(false), tf_buffer_(NULL), costmap_ros_(NULL), odom_helper_("odom") {}

// Pluginler genellikle varsayılan constructor ile oluşturulur ve sonra initialize çağrılır.
// Bu ikinci constructor genellikle doğrudan kullanılmaz.
PurePursuitLocalPlanner::PurePursuitLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : initialized_(false), goal_reached_(false), tf_buffer_(tf), costmap_ros_(costmap_ros), odom_helper_("odom") {
    // initialize(name, tf, costmap_ros); // Genellikle initialize burada çağrılmaz.
}

PurePursuitLocalPlanner::~PurePursuitLocalPlanner() {}

void PurePursuitLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        tf_buffer_ = tf;
        costmap_ros_ = costmap_ros;
        if (!costmap_ros_) {
            ROS_FATAL("[%s] Costmap ROS Wrapper is NULL in initialize! Cannot operate.", name.c_str());
            return;
        }
        if (!tf_buffer_){
            ROS_FATAL("[%s] TF Buffer is NULL in initialize! Cannot operate.", name.c_str());
            return;
        }

        ros::NodeHandle private_nh("~/" + name);

        private_nh.param("lookahead_distance", lookahead_distance_, 0.5);
        private_nh.param("linear_vel", linear_vel_, 0.15);
        private_nh.param("max_angular_vel", max_angular_vel_, 0.7);
        private_nh.param("goal_dist_tolerance", goal_dist_tolerance_, 0.15);
        private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_footprint"));
        
        // Global frame'i "map" olarak sabitleyelim.
        // Planlar genellikle "map" çerçevesinde gelir ve robotun pozisyonu da bu çerçeveye dönüştürülmelidir.
        global_frame_ = "map"; 
        ROS_INFO("[%s] Using hardcoded global frame for Pure Pursuit: '%s'", name.c_str(), global_frame_.c_str());
        // Alternatif: costmap'ten al ama yerel costmap global_frame'i odom olabilir.
        // std::string costmap_g_frame = costmap_ros_->getGlobalFrameID();
        // ROS_INFO("[%s] Costmap's global_frame: '%s'. Pure Pursuit will use '%s'.", name.c_str(), costmap_g_frame.c_str(), global_frame_.c_str());


        odom_helper_.setOdomTopic(ros::NodeHandle().resolveName("odom"));
        carrot_pub_ = private_nh.advertise<visualization_msgs::Marker>("carrot_marker", 1, true);
        
        initialized_ = true;
        ROS_INFO("[%s] PurePursuitLocalPlanner initialized. Lookahead: %.2f, LinVel: %.2f, MaxAngVel: %.2f, GoalTol: %.2f, BaseFrame: %s, GlobalFrame: %s",
                 name.c_str(), lookahead_distance_, linear_vel_, max_angular_vel_, goal_dist_tolerance_, robot_base_frame_.c_str(), global_frame_.c_str());
    } else {
        ROS_WARN("[%s] PurePursuitLocalPlanner has already been initialized.", name.c_str());
    }
}

bool PurePursuitLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (!initialized_) {
        ROS_ERROR("PurePursuitLocalPlanner: Not initialized, cannot set plan.");
        return false;
    }
    global_plan_ = orig_global_plan;
    current_segment_idx_ = 0;
    goal_reached_ = false;

    if (global_plan_.empty()) {
        ROS_WARN("PurePursuitLocalPlanner: Received an empty plan. Setting goal_reached to true.");
        goal_reached_ = true; // Boş plan, yapılacak bir şey yok.
    } else {
        ROS_INFO("PurePursuitLocalPlanner: Received new plan with %zu points. First point frame: '%s', Last point frame: '%s'. Expected global frame for planner: '%s'",
                 global_plan_.size(),
                 global_plan_.front().header.frame_id.c_str(),
                 global_plan_.back().header.frame_id.c_str(),
                 global_frame_.c_str());
        // Planın tüm noktalarının beklenen global_frame_'de olduğunu kontrol et (opsiyonel ama iyi bir pratik)
        for(const auto& pose : global_plan_){
            if(pose.header.frame_id != global_frame_){
                ROS_ERROR("PurePursuitLocalPlanner: Plan contains poses in frame '%s' which MISMATCHES expected global frame '%s'! This will cause TF errors. Aborting setPlan.",
                         pose.header.frame_id.c_str(), global_frame_.c_str());
                global_plan_.clear(); // Geçersiz planı temizle
                goal_reached_ = true; // Hatalı plan, devam etme
                return false;
            }
        }
    }
    return true;
}

bool PurePursuitLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

    if (!initialized_) {
        ROS_ERROR_THROTTLE(1.0, "PurePursuitLocalPlanner: Not initialized.");
        return false;
    }
    if (global_plan_.empty()) {
        ROS_DEBUG_THROTTLE(1.0, "PurePursuitLocalPlanner: Plan is empty.");
        if(!goal_reached_) goal_reached_ = true; // Emin olmak için
        return true; // Hedefe ulaşıldı sayılır
    }
    if (goal_reached_) {
        ROS_DEBUG_THROTTLE(1.0, "PurePursuitLocalPlanner: Goal already reached.");
        return true;
    }

    geometry_msgs::PoseStamped robot_pose_from_costmap;
    if (!costmap_ros_->getRobotPose(robot_pose_from_costmap)) {
        ROS_ERROR_THROTTLE(1.0, "PurePursuitLocalPlanner: Failed to get robot pose from costmap.");
        return false;
    }

    geometry_msgs::PoseStamped robot_pose_in_global_frame = robot_pose_from_costmap;
    if (robot_pose_from_costmap.header.frame_id != global_frame_) {
        ROS_DEBUG_THROTTLE(2.0, "PurePursuitLocalPlanner: Robot pose from costmap (frame: '%s') is not in the global frame ('%s'). Transforming...",
                         robot_pose_from_costmap.header.frame_id.c_str(), global_frame_.c_str());
        try {
            geometry_msgs::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
                global_frame_, robot_pose_from_costmap.header.frame_id, ros::Time(0), ros::Duration(0.2));
            tf2::doTransform(robot_pose_from_costmap, robot_pose_in_global_frame, tf_stamped);
            robot_pose_in_global_frame.header.frame_id = global_frame_; // Çerçeveyi güncelle
        } catch (tf2::TransformException &ex) {
            ROS_ERROR_THROTTLE(1.0, "PurePursuitLocalPlanner: TF Exception transforming robot pose from '%s' to '%s': %s",
                      robot_pose_from_costmap.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
            return false;
        }
    }

    const geometry_msgs::PoseStamped& final_goal_on_plan = global_plan_.back();
    double dist_to_final_goal = std::hypot(final_goal_on_plan.pose.position.x - robot_pose_in_global_frame.pose.position.x,
                                           final_goal_on_plan.pose.position.y - robot_pose_in_global_frame.pose.position.y);

    if (dist_to_final_goal < goal_dist_tolerance_) {
        ROS_INFO("PurePursuitLocalPlanner: Goal reached! Distance to final plan point: %.3f m", dist_to_final_goal);
        goal_reached_ = true;
        return true;
    }
    
    geometry_msgs::PoseStamped lookahead_point_in_global_frame = findLookaheadPoint(robot_pose_in_global_frame);

    if (lookahead_point_in_global_frame.header.frame_id.empty() || lookahead_point_in_global_frame.header.frame_id != global_frame_) {
        ROS_WARN_THROTTLE(1.0, "PurePursuitLocalPlanner: No valid lookahead point found in '%s' frame. Targeting final goal if close enough.",
                         global_frame_.c_str());
        if (dist_to_final_goal < lookahead_distance_ * 1.5) { // Biraz daha geniş bir toleransla son hedefi kovala
            lookahead_point_in_global_frame = final_goal_on_plan;
            lookahead_point_in_global_frame.header.frame_id = global_frame_; // Çerçeveyi garantile
        } else {
            ROS_WARN_THROTTLE(1.0, "PurePursuitLocalPlanner: Final goal too far, stopping.");
            return false; 
        }
    }
    
    publishCarrotMarker(lookahead_point_in_global_frame.pose.position);

    geometry_msgs::PoseStamped lookahead_point_in_robot_frame;
    try {
        geometry_msgs::TransformStamped tf_global_to_robot = tf_buffer_->lookupTransform(
            robot_base_frame_, global_frame_, ros::Time(0), ros::Duration(0.1));
        tf2::doTransform(lookahead_point_in_global_frame, lookahead_point_in_robot_frame, tf_global_to_robot);
    } catch (tf2::TransformException& ex) {
        ROS_ERROR_THROTTLE(1.0, "PurePursuitLocalPlanner: TF Exception transforming lookahead point from '%s' to '%s': %s",
                         global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
        return false;
    }

    double x_local_carrot = lookahead_point_in_robot_frame.pose.position.x;
    double y_local_carrot = lookahead_point_in_robot_frame.pose.position.y;
    
    double alpha_to_carrot = atan2(y_local_carrot, x_local_carrot);
    double actual_dist_to_carrot = std::hypot(x_local_carrot, y_local_carrot);
    
    if (actual_dist_to_carrot < 0.01) {
        cmd_vel.linear.x = linear_vel_ * 0.3; // Çok yakınsa yavaşça ileri git
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

geometry_msgs::PoseStamped PurePursuitLocalPlanner::findLookaheadPoint(const geometry_msgs::PoseStamped& robot_pose_in_global_frame) {
    geometry_msgs::PoseStamped lookahead_pt;
    lookahead_pt.header.frame_id = ""; // Varsayılan olarak geçersiz

    if (global_plan_.empty()) {
        ROS_WARN_THROTTLE(1.0, "findLookaheadPoint: Global plan is empty, cannot find lookahead point.");
        return lookahead_pt;
    }

    // Hedef noktanın çerçevesi, global_plan_'ın çerçevesiyle (veya bizim beklediğimiz global_frame_ ile) aynı olmalı
    lookahead_pt.header.frame_id = global_frame_;


    // current_segment_idx_'dan başlayarak yolu tara
    for (size_t i = current_segment_idx_; i < global_plan_.size(); ++i) {
        // Yolun sonuna ulaşıldı mı?
        if (i + 1 >= global_plan_.size()) { 
            // Son nokta, potansiyel lookahead noktasıdır
            double dist_sq_to_last = pow(global_plan_[i].pose.position.x - robot_pose_in_global_frame.pose.position.x, 2) +
                                     pow(global_plan_[i].pose.position.y - robot_pose_in_global_frame.pose.position.y, 2);
            if (dist_sq_to_last <= pow(lookahead_distance_ * 1.1, 2)) { // Son nokta lookahead çemberinin biraz genişletilmiş halinde mi?
                current_segment_idx_ = i; // İlerlemeyi güncelle
                lookahead_pt = global_plan_[i];
                // ROS_DEBUG("findLookaheadPoint: Targeting last point of the plan as lookahead.");
                return lookahead_pt;
            }
            // Eğer son nokta lookahead dışında kalıyorsa, bir önceki geçerli kesişim kullanılmalıydı veya hata
            break; // Başka segment yok
        }

        // (p1, p2) segmenti
        const geometry_msgs::Point& p1 = global_plan_[i].pose.position;
        const geometry_msgs::Point& p2 = global_plan_[i+1].pose.position;
        const geometry_msgs::Point& robot_p = robot_pose_in_global_frame.pose.position;

        // Segment vektörü: d_vec = p2 - p1
        double dx_seg = p2.x - p1.x;
        double dy_seg = p2.y - p1.y;

        // p1'den robota olan vektör: f_vec = robot_p - p1 (veya p1 - robot_p, işarete dikkat)
        // Kuadratik denklem için: (robot_p - (p1 + t*d_vec))^2 = Ld^2
        // ( (robot_p.x - p1.x) - t*dx_seg )^2 + ( (robot_p.y - p1.y) - t*dy_seg )^2 = Ld^2
        // Let drx = robot_p.x - p1.x, dry = robot_p.y - p1.y
        // (drx - t*dx_seg)^2 + (dry - t*dy_seg)^2 = Ld^2
        // drx^2 - 2*t*drx*dx_seg + t^2*dx_seg^2 + dry^2 - 2*t*dry*dy_seg + t^2*dy_seg^2 = Ld^2
        // t^2 * (dx_seg^2 + dy_seg^2) + t * (-2*drx*dx_seg - 2*dry*dy_seg) + (drx^2 + dry^2 - Ld^2) = 0
        // a*t^2 + b*t + c = 0

        double drx = robot_p.x - p1.x;
        double dry = robot_p.y - p1.y;

        double a_quad = dx_seg * dx_seg + dy_seg * dy_seg;
        double b_quad = -2 * (drx * dx_seg + dry * dy_seg); // Dikkat: formül (-b +/- ...), b'nin işareti önemli
        double c_quad = drx * drx + dry * dry - lookahead_distance_ * lookahead_distance_;
        
        double discriminant = b_quad * b_quad - 4 * a_quad * c_quad;

        if (discriminant >= 0) {
            if (std::abs(a_quad) < 1e-9) { // Segment çok kısa (p1 ve p2 aynı nokta gibi)
                // Sadece p1'in lookahead çemberi içinde olup olmadığını kontrol et
                if (c_quad <= 0) { // drx^2 + dry^2 <= Ld^2
                     lookahead_pt = global_plan_[i]; // p1'i hedefle
                     current_segment_idx_ = i;
                     return lookahead_pt;
                }
                continue; // Bu segmentte işlem yapma
            }

            discriminant = sqrt(discriminant);
            double t1 = (-b_quad - discriminant) / (2 * a_quad);
            double t2 = (-b_quad + discriminant) / (2 * a_quad);
            
            // Segment üzerindeki geçerli t değerlerini (0 <= t <= 1) ve ileride olanı seç
            bool t1_valid = (t1 >= -1e-3 && t1 <= 1.0 + 1e-3); // Küçük toleranslar
            bool t2_valid = (t2 >= -1e-3 && t2 <= 1.0 + 1e-3);

            double valid_t = -1.0;

            if (t1_valid && t2_valid) {
                valid_t = std::max(t1, t2); // İleride olanı al
            } else if (t1_valid) {
                valid_t = t1;
            } else if (t2_valid) {
                valid_t = t2;
            }

            if (valid_t >= -1e-3) { // Geçerli bir t bulunduysa
                valid_t = std::max(0.0, std::min(1.0, valid_t)); // t'yi [0,1] aralığına klample
                lookahead_pt.pose.position.x = p1.x + valid_t * dx_seg;
                lookahead_pt.pose.position.y = p1.y + valid_t * dy_seg;
                lookahead_pt.pose.position.z = p1.z; // Veya (p1.z + valid_t * (p2.z - p1.z))
                // Yönelim olarak segmentin sonundaki noktanın yönelimini veya enterpolasyon yapılabilir
                lookahead_pt.pose.orientation = global_plan_[i+1].pose.orientation;
                current_segment_idx_ = i; // İlerlemeyi bu segmente güncelle
                // ROS_DEBUG("findLookaheadPoint: Found lookahead on segment %zu with t=%.3f", i, valid_t);
                return lookahead_pt;
            }
        }
    }
    
    // Eğer döngü bitti ve hala bir kesişim bulunamadıysa (örn: robot yoldan çok saptı),
    // veya yolun sonuna çok yaklaşıldı ve son nokta lookahead çemberi dışında kaldı.
    // Bu durumda, yolun son noktasını hedef olarak ata.
    if (!global_plan_.empty()) {
        ROS_WARN_THROTTLE(1.0, "findLookaheadPoint: No valid intersection found on path segments. Targeting the last point of the plan.");
        lookahead_pt = global_plan_.back();
        current_segment_idx_ = global_plan_.size() - 1; // Son segmente güncelle
        return lookahead_pt;
    }
    
    ROS_WARN_THROTTLE(1.0, "findLookaheadPoint: Could not find any lookahead point and plan is empty/invalid.");
    lookahead_pt.header.frame_id = ""; // Geçersiz olarak işaretle
    return lookahead_pt;
}


bool PurePursuitLocalPlanner::isGoalReached() {
    return goal_reached_;
}

void PurePursuitLocalPlanner::publishCarrotMarker(const geometry_msgs::Point& carrot_point_global_frame) {
    if (carrot_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::Marker marker;
    marker.header.frame_id = global_frame_; // Marker'ı her zaman beklenen global_frame_'de yayınla
    marker.header.stamp = ros::Time::now();
    marker.ns = "pure_pursuit_carrot";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position = carrot_point_global_frame;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2;
    marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; marker.color.a = 0.8; // Biraz şeffaflık
    marker.lifetime = ros::Duration(0.5);
    
    carrot_pub_.publish(marker);
}

} // namespace blm6191_coverage_planners
