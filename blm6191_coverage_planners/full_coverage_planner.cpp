#include "full_coverage_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <algorithm>
#include <vector>
#include <sstream>
#include <cmath>
#include <unordered_set>
#include <limits>

PLUGINLIB_EXPORT_CLASS(blm6191_coverage_planners::FullCoveragePlanner, nav_core::BaseGlobalPlanner)

namespace blm6191_coverage_planners {

FullCoveragePlanner::FullCoveragePlanner() 
    : initialized_(false), costmap_ros_(nullptr), costmap_(nullptr) {}

FullCoveragePlanner::FullCoveragePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : initialized_(false), costmap_ros_(nullptr), costmap_(nullptr) {
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
        
        // Original parameters
        private_nh.param("robot_radius", robot_radius_, 0.17);
        private_nh.param("sweep_spacing_factor", sweep_spacing_factor_, 0.9);
        private_nh.param("path_point_distance", path_point_distance_, 0.1);
        private_nh.param("frame_id", frame_id_, std::string("map"));
        private_nh.param("use_param_poly", use_param_poly_, true);
        
        // Enhanced obstacle avoidance parameters
        private_nh.param("safety_margin", safety_margin_, 0.08);
        private_nh.param("max_obstacle_distance", max_obstacle_distance_, 2.0);
        private_nh.param("max_detour_attempts", max_detour_attempts_, 5);
        private_nh.param("adaptive_spacing_factor", adaptive_spacing_factor_, 1.3);
        private_nh.param("use_dynamic_spacing", use_dynamic_spacing_, true);
        private_nh.param("obstacle_check_resolution", obstacle_check_resolution_, 0.02);
        private_nh.param("line_of_sight_resolution", line_of_sight_resolution_, 0.05);

        // Obstacle coverage parameters - YENİ!
        private_nh.param("enable_obstacle_coverage", enable_obstacle_coverage_, true);
        private_nh.param("obstacle_coverage_radius", obstacle_coverage_radius_, 1.0);
        private_nh.param("min_coverage_area", min_coverage_area_, 0.25); // 0.5m x 0.5m minimum alan
        private_nh.param("coverage_sweep_directions", coverage_sweep_directions_, 8);

        // Validate parameters
        if (robot_radius_ <= 0) {
            ROS_WARN("[%s] Invalid robot_radius: %f. Setting to default 0.17", name.c_str(), robot_radius_);
            robot_radius_ = 0.17;
        }
        
        if (safety_margin_ < 0) {
            ROS_WARN("[%s] Invalid safety_margin: %f. Setting to default 0.08", name.c_str(), safety_margin_);
            safety_margin_ = 0.08;
        }

        if (use_param_poly_) {
            // Set default polygon corners to a 6x6 square centered at origin
            private_nh.param("p1_x", p1x_, -3.0); private_nh.param("p1_y", p1y_, -3.0);
            private_nh.param("p2_x", p2x_, 3.0); private_nh.param("p2_y", p2y_, -3.0);
            private_nh.param("p3_x", p3x_, 3.0); private_nh.param("p3_y", p3y_, 3.0);
            private_nh.param("p4_x", p4x_, -3.0); private_nh.param("p4_y", p4y_, 3.0);

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
        obstacle_regions_pub_ = private_nh.advertise<nav_msgs::Path>("obstacle_coverage_regions", 1);
        initialized_ = true;
        
        ROS_INFO("[%s] FullCoveragePlanner initialized with COMPREHENSIVE obstacle coverage!", name.c_str());
        ROS_INFO("[%s] Robot radius: %.3f, Safety margin: %.3f", name.c_str(), robot_radius_, safety_margin_);
        ROS_INFO("[%s] Obstacle coverage: %s, Coverage radius: %.2f", 
                 name.c_str(), enable_obstacle_coverage_ ? "ENABLED" : "disabled", obstacle_coverage_radius_);
    }
}

void FullCoveragePlanner::polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg) {
    if (!use_param_poly_) {
        if (msg->polygon.points.size() == 4) {
            polygon_corners_msg_ = msg->polygon.points;
            ROS_INFO("FullCoveragePlanner: Received polygon with 4 corners via topic.");
        } else {
            ROS_WARN("FullCoveragePlanner: Received polygon via topic but it does not have 4 corners (%zu). Ignoring.", 
                msg->polygon.points.size());
            polygon_corners_msg_.clear();
        }
    }
}

bool FullCoveragePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("[FullCoveragePlanner] Planner not initialized.");
        return false;
    }
    
    if (polygon_corners_msg_.size() != 4) {
        ROS_ERROR("[FullCoveragePlanner] Polygon not defined (4 corners needed, got %zu).", 
                  polygon_corners_msg_.size());
        return false;
    }
    
    if (!costmap_) {
        ROS_ERROR("[FullCoveragePlanner] Costmap not available.");
        return false;
    }

    plan.clear();
    
    // Convert polygon corners
    std::vector<geometry_msgs::Point> poly_corners_geom(polygon_corners_msg_.size());
    for(size_t i = 0; i < polygon_corners_msg_.size(); ++i) {
        poly_corners_geom[i].x = polygon_corners_msg_[i].x;
        poly_corners_geom[i].y = polygon_corners_msg_[i].y;
        poly_corners_geom[i].z = 0.0;
    }

    ROS_INFO("[FullCoveragePlanner] Generating COMPREHENSIVE coverage path with obstacle coverage...");
    
    // ADIM 1: Ana boustrophedon yolu oluştur
    std::vector<geometry_msgs::PoseStamped> main_plan;
    generateBoustrophedonPath(poly_corners_geom, start, main_plan);
    
    // ADIM 2: Engel bölgelerini tespit et ve kapsayıcı plan oluştur
    if (enable_obstacle_coverage_ && !main_plan.empty()) {
        std::vector<ObstacleRegion> obstacle_regions = detectObstacleRegions(poly_corners_geom);
        
        ROS_INFO("[FullCoveragePlanner] Detected %zu obstacle regions for coverage", obstacle_regions.size());
        
        for (const auto& obstacle_region : obstacle_regions) {
            std::vector<geometry_msgs::PoseStamped> obstacle_coverage;
            generateObstacleCoverage(obstacle_region, poly_corners_geom, obstacle_coverage);
            
            if (!obstacle_coverage.empty()) {
                // En yakın ana yol noktasını bul ve engel kapsamını entegre et
                geometry_msgs::Point closest_main_point;
                double min_distance = std::numeric_limits<double>::max();
                
                for (const auto& main_pose : main_plan) {
                    double dist = euclideanDistance(main_pose.pose.position, obstacle_region.center);
                    if (dist < min_distance) {
                        min_distance = dist;
                        closest_main_point = main_pose.pose.position;
                    }
                }
                
                integrateObstacleCoverageIntoPlan(main_plan, obstacle_coverage, closest_main_point);
                ROS_INFO("[FullCoveragePlanner] Integrated obstacle coverage with %zu points", obstacle_coverage.size());
            }
        }
    }
    
    plan = main_plan;

    if (!plan.empty()) {
        // Path validation and refinement
        if (!validatePath(plan)) {
            ROS_WARN("[FullCoveragePlanner] Path validation failed, refining path...");
            plan = refinePath(plan);
        }
        
        // Apply path smoothing
        std::vector<geometry_msgs::PoseStamped> smoothed_plan = smoothPath(plan);
        if (!smoothed_plan.empty()) {
            plan = smoothed_plan;
        }
        
        // Verify coverage completeness
        if (enable_obstacle_coverage_) {
            std::vector<geometry_msgs::Point> uncovered_areas = findUncoveredAreas(plan, poly_corners_geom);
            if (!uncovered_areas.empty()) {
                ROS_WARN("[FullCoveragePlanner] Found %zu uncovered areas, consider adjusting parameters", 
                         uncovered_areas.size());
            }
        }
        
        // Publish the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(plan.size());
        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();
        
        for (size_t i = 0; i < plan.size(); i++) {
            plan[i].header.frame_id = frame_id_;
            plan[i].header.stamp = ros::Time::now();
            gui_path.poses[i] = plan[i];
        }
        
        plan_pub_.publish(gui_path);
        ROS_INFO("[FullCoveragePlanner] COMPREHENSIVE coverage plan generated with %zu points!", plan.size());
        return true;
    } else {
        ROS_ERROR("[FullCoveragePlanner] Failed to generate coverage plan.");
        return false;
    }
}

// Engel bölgelerini tespit et
std::vector<ObstacleRegion> FullCoveragePlanner::detectObstacleRegions(
    const std::vector<geometry_msgs::Point>& polygon) {
    
    std::vector<ObstacleRegion> regions;
    
    if (!costmap_) return regions;
    
    // Poligon sınırlarını hesapla
    double min_x = polygon[0].x, max_x = polygon[0].x;
    double min_y = polygon[0].y, max_y = polygon[0].y;
    
    for (const auto& point : polygon) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    
    // Grid taraması ile engel merkezlerini bul
    double resolution = costmap_->getResolution();
    double search_step = robot_radius_ * 1.5; // Daha sık tarama
    
    std::set<std::string> processed_obstacles;
    
    for (double x = min_x; x <= max_x; x += search_step) {
        for (double y = min_y; y <= max_y; y += search_step) {
            geometry_msgs::Point candidate_center = createPoint(x, y, 0.0);
            
            // Poligon içinde ve engel olan nokta mı?
            if (isPointInPolygon(candidate_center, polygon) && isObstacle(x, y)) {
                
                std::string obstacle_key = pointToString(x, y);
                if (processed_obstacles.find(obstacle_key) != processed_obstacles.end()) {
                    continue; // Bu engel zaten işlendi
                }
                
                // Engelin detaylı konturunu bul
                std::vector<geometry_msgs::Point> detailed_contour = 
                    getDetailedObstacleContour(x, y, obstacle_coverage_radius_);
                
                if (detailed_contour.size() >= 3) { // Geçerli bir engel
                    ObstacleRegion region;
                    region.center = candidate_center;
                    region.contour_points = detailed_contour;
                    region.max_radius = obstacle_coverage_radius_;
                    
                    // Bu engel etrafındaki erişilebilir alanları bul
                    std::vector<geometry_msgs::Point> accessible_areas = 
                        findAccessibleAreasAroundObstacle(candidate_center, obstacle_coverage_radius_, polygon);
                    
                    // Erişilebilir alan yeterince büyük mü?
                    if (calculateAreaSize(accessible_areas) >= min_coverage_area_) {
                        // Alt bölgelere ayır
                        region.sub_regions = subdivideArea(accessible_areas);
                        regions.push_back(region);
                        
                        // İşlenmiş olarak işaretle (yakın noktaları da)
                        for (double dx = -search_step; dx <= search_step; dx += resolution) {
                            for (double dy = -search_step; dy <= search_step; dy += resolution) {
                                std::string nearby_key = pointToString(x + dx, y + dy);
                                processed_obstacles.insert(nearby_key);
                            }
                        }
                    }
                }
            }
        }
    }
    
    ROS_INFO("[FullCoveragePlanner] Detected %zu significant obstacle regions requiring coverage", regions.size());
    return regions;
}

// Engel etrafındaki erişilebilir alanları bul
std::vector<geometry_msgs::Point> FullCoveragePlanner::findAccessibleAreasAroundObstacle(
    const geometry_msgs::Point& obstacle_center, 
    double search_radius,
    const std::vector<geometry_msgs::Point>& polygon) {
    
    std::vector<geometry_msgs::Point> accessible_points;
    
    double resolution = costmap_->getResolution();
    int steps = static_cast<int>(search_radius / resolution);
    
    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            double check_x = obstacle_center.x + dx * resolution;
            double check_y = obstacle_center.y + dy * resolution;
            
            double distance = sqrt(dx*dx + dy*dy) * resolution;
            
            // Arama yarıçapı içinde mi?
            if (distance <= search_radius && distance >= robot_radius_ + safety_margin_) {
                geometry_msgs::Point check_point = createPoint(check_x, check_y, 0.0);
                
                // Poligon içinde, güvenli ve erişilebilir mi?
                if (isPointInPolygon(check_point, polygon) && 
                    isPointSafe(check_x, check_y, robot_radius_) &&
                    !isObstacle(check_x, check_y)) {
                    accessible_points.push_back(check_point);
                }
            }
        }
    }
    
    return accessible_points;
}

// Engel kapsama planı oluştur
void FullCoveragePlanner::generateObstacleCoverage(
    const ObstacleRegion& obstacle_region,
    const std::vector<geometry_msgs::Point>& polygon,
    std::vector<geometry_msgs::PoseStamped>& coverage_plan) {
    
    coverage_plan.clear();
    
    // Üç farklı kapsama desenini dene ve en iyisini seç
    std::vector<geometry_msgs::PoseStamped> radial_coverage, spiral_coverage, perimeter_coverage;
    
    // 1. Radyal kapsama (merkezden dışarı)
    std::vector<geometry_msgs::Point> accessible_areas = 
        findAccessibleAreasAroundObstacle(obstacle_region.center, obstacle_region.max_radius, polygon);
    
    if (!accessible_areas.empty()) {
        radial_coverage = generateRadialCoverage(obstacle_region.center, accessible_areas, polygon);
        spiral_coverage = generateSpiralCoverage(obstacle_region.center, accessible_areas, polygon);
    }
    
    // 2. Perimeter kapsama (çevre takibi)
    if (!obstacle_region.contour_points.empty()) {
        perimeter_coverage = generatePerimeterCoverage(obstacle_region.contour_points, polygon);
    }
    
    // En kapsamlı olanı seç
    std::vector<geometry_msgs::PoseStamped>* best_coverage = &radial_coverage;
    
    if (spiral_coverage.size() > best_coverage->size()) {
        best_coverage = &spiral_coverage;
    }
    if (perimeter_coverage.size() > best_coverage->size()) {
        best_coverage = &perimeter_coverage;
    }
    
    coverage_plan = *best_coverage;
    
    ROS_DEBUG("[FullCoveragePlanner] Generated obstacle coverage: Radial=%zu, Spiral=%zu, Perimeter=%zu, Selected=%zu", 
              radial_coverage.size(), spiral_coverage.size(), perimeter_coverage.size(), coverage_plan.size());
}

// Radyal kapsama deseni
std::vector<geometry_msgs::PoseStamped> FullCoveragePlanner::generateRadialCoverage(
    const geometry_msgs::Point& center,
    const std::vector<geometry_msgs::Point>& accessible_area,
    const std::vector<geometry_msgs::Point>& polygon) {
    
    std::vector<geometry_msgs::PoseStamped> radial_plan;
    
    if (accessible_area.empty()) return radial_plan;
    
    // Radyal tarama: Merkez etrafında açısal süpürme
    int num_directions = coverage_sweep_directions_;
    double angle_step = 2.0 * M_PI / num_directions;
    
    for (int dir = 0; dir < num_directions; ++dir) {
        double angle = dir * angle_step;
        double cos_angle = cos(angle);
        double sin_angle = sin(angle);
        
        // Bu yönde mümkün olduğunca uzakta git
        double max_radius = 0.0;
        for (const auto& accessible_point : accessible_area) {
            double dx = accessible_point.x - center.x;
            double dy = accessible_point.y - center.y;
            
            // Bu yön vektorüyle aynı yönde mi?
            double dot_product = dx * cos_angle + dy * sin_angle;
            if (dot_product > 0) {
                double radius = sqrt(dx*dx + dy*dy);
                max_radius = std::max(max_radius, radius);
            }
        }
        
        // Bu yönde robot yarıçapından başlayarak tarama yap
        for (double r = robot_radius_ + safety_margin_; r <= max_radius; r += path_point_distance_) {
            double sweep_x = center.x + r * cos_angle;
            double sweep_y = center.y + r * sin_angle;
            
            geometry_msgs::Point sweep_point = createPoint(sweep_x, sweep_y, 0.0);
            
            if (isPointInPolygon(sweep_point, polygon) && 
                isPointSafe(sweep_x, sweep_y, robot_radius_)) {
                
                // Yönelim: merkezden dışarı doğru
                double orientation = angle;
                radial_plan.push_back(createPoseStamped(sweep_point, orientation));
            }
        }
    }
    
    return radial_plan;
}

// Spiral kapsama deseni
std::vector<geometry_msgs::PoseStamped> FullCoveragePlanner::generateSpiralCoverage(
    const geometry_msgs::Point& center,
    const std::vector<geometry_msgs::Point>& accessible_area,
    const std::vector<geometry_msgs::Point>& polygon) {
    
    std::vector<geometry_msgs::PoseStamped> spiral_plan;
    
    if (accessible_area.empty()) return spiral_plan;
    
    // Spiral: İçten dışa doğru spiral hareket
    double max_radius = 0.0;
    for (const auto& point : accessible_area) {
        double radius = euclideanDistance(center, point);
        max_radius = std::max(max_radius, radius);
    }
    
    double spiral_step = robot_radius_ * sweep_spacing_factor_;
    double angle_increment = spiral_step / (robot_radius_ + safety_margin_);
    
    double current_radius = robot_radius_ + safety_margin_;
    double current_angle = 0.0;
    
    while (current_radius <= max_radius) {
        double spiral_x = center.x + current_radius * cos(current_angle);
        double spiral_y = center.y + current_radius * sin(current_angle);
        
        geometry_msgs::Point spiral_point = createPoint(spiral_x, spiral_y, 0.0);
        
        if (isPointInPolygon(spiral_point, polygon) && 
            isPointSafe(spiral_x, spiral_y, robot_radius_)) {
            
            // Yönelim: spiral yönü
            double orientation = current_angle + M_PI/2; // Teğet yön
            spiral_plan.push_back(createPoseStamped(spiral_point, orientation));
        }
        
        // Spiral parametrelerini güncelle
        current_angle += angle_increment;
        current_radius += spiral_step * angle_increment / (2.0 * M_PI);
    }
    
    return spiral_plan;
}

// Perimeter kapsama deseni
std::vector<geometry_msgs::PoseStamped> FullCoveragePlanner::generatePerimeterCoverage(
    const std::vector<geometry_msgs::Point>& obstacle_contour,
    const std::vector<geometry_msgs::Point>& polygon) {
    
    std::vector<geometry_msgs::PoseStamped> perimeter_plan;
    
    if (obstacle_contour.size() < 3) return perimeter_plan;
    
    // Engel çevresini takip et
    double follow_distance = robot_radius_ + safety_margin_ * 2.0;
    
    for (size_t i = 0; i < obstacle_contour.size(); ++i) {
        const auto& contour_point = obstacle_contour[i];
        const auto& next_contour_point = obstacle_contour[(i + 1) % obstacle_contour.size()];
        
        // Kontür noktasından dışarı doğru normal vektör hesapla
        double edge_dx = next_contour_point.x - contour_point.x;
        double edge_dy = next_contour_point.y - contour_point.y;
        double edge_length = sqrt(edge_dx*edge_dx + edge_dy*edge_dy);
        
        if (edge_length > 1e-6) {
            // Sağa dönük normal vektör
            double normal_x = -edge_dy / edge_length;
            double normal_y = edge_dx / edge_length;
            
            double follow_x = contour_point.x + normal_x * follow_distance;
            double follow_y = contour_point.y + normal_y * follow_distance;
            
            geometry_msgs::Point follow_point = createPoint(follow_x, follow_y, 0.0);
            
            if (isPointInPolygon(follow_point, polygon) && 
                isPointSafe(follow_x, follow_y, robot_radius_)) {
                
                // Yönelim: kontür boyunca
                double orientation = atan2(edge_dy, edge_dx);
                perimeter_plan.push_back(createPoseStamped(follow_point, orientation));
            }
        }
    }
    
    return perimeter_plan;
}

// Engel kapsamını ana plana entegre et
void FullCoveragePlanner::integrateObstacleCoverageIntoPlan(
    std::vector<geometry_msgs::PoseStamped>& main_plan,
    const std::vector<geometry_msgs::PoseStamped>& obstacle_coverage,
    const geometry_msgs::Point& connection_point) {
    
    if (obstacle_coverage.empty() || main_plan.empty()) return;
    
    // Ana plandaki en yakın noktayı bul
    auto closest_it = main_plan.begin();
    double min_distance = std::numeric_limits<double>::max();
    
    for (auto it = main_plan.begin(); it != main_plan.end(); ++it) {
        double dist = euclideanDistance(it->pose.position, connection_point);
        if (dist < min_distance) {
            min_distance = dist;
            closest_it = it;
        }
    }
    
    // Bağlantı yolunu oluştur
    std::vector<geometry_msgs::Point> polygon; // Ana poligon verisi gerekli
    for (size_t i = 0; i < polygon_corners_msg_.size(); ++i) {
        polygon.push_back(createPoint(polygon_corners_msg_[i].x, polygon_corners_msg_[i].y, 0.0));
    }
    
    std::vector<geometry_msgs::PoseStamped> connection_to_obstacle = 
        createConnectionPath(closest_it->pose.position, obstacle_coverage[0].pose.position, 0.0, polygon);
    
    std::vector<geometry_msgs::PoseStamped> connection_from_obstacle = 
        createConnectionPath(obstacle_coverage.back().pose.position, closest_it->pose.position, 0.0, polygon);
    
    // Ana plana ekle
    auto insert_pos = closest_it + 1;
    
    // Engel kapsamına gidiş
    main_plan.insert(insert_pos, connection_to_obstacle.begin(), connection_to_obstacle.end());
    insert_pos += connection_to_obstacle.size();
    
    // Engel kapsamı
    main_plan.insert(insert_pos, obstacle_coverage.begin(), obstacle_coverage.end());
    insert_pos += obstacle_coverage.size();
    
    // Engel kapsamından dönüş
    main_plan.insert(insert_pos, connection_from_obstacle.begin(), connection_from_obstacle.end());
}

// Alan büyüklüğü hesapla
double FullCoveragePlanner::calculateAreaSize(const std::vector<geometry_msgs::Point>& area_points) {
    if (area_points.empty()) return 0.0;
    
    // Basit yaklaşım: sınırlayıcı kutu alanı
    double min_x = area_points[0].x, max_x = area_points[0].x;
    double min_y = area_points[0].y, max_y = area_points[0].y;
    
    for (const auto& point : area_points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    
    return (max_x - min_x) * (max_y - min_y);
}

// Alanı alt bölgelere ayır
std::vector<std::vector<geometry_msgs::Point>> FullCoveragePlanner::subdivideArea(
    const std::vector<geometry_msgs::Point>& area_points) {
    
    std::vector<std::vector<geometry_msgs::Point>> sub_regions;
    
    // Şimdilik tek bölge olarak döndür, gelecekte daha karmaşık bölme algoritması eklenebilir
    if (!area_points.empty()) {
        sub_regions.push_back(area_points);
    }
    
    return sub_regions;
}

// Detaylı engel konturu
std::vector<geometry_msgs::Point> FullCoveragePlanner::getDetailedObstacleContour(
    double center_x, double center_y, double search_radius) {
    
    std::vector<geometry_msgs::Point> detailed_contour;
    
    if (!costmap_) return detailed_contour;
    
    int num_points = 36; // Daha detaylı kontür için daha fazla nokta
    
    for (int i = 0; i < num_points; ++i) {
        double angle = 2.0 * M_PI * i / num_points;
        
        // Binary search ile engel sınırını bul
        double min_radius = 0.0;
        double max_radius = search_radius;
        double best_radius = 0.0;
        
        for (int j = 0; j < 15; ++j) { // Daha hassas binary search
            double test_radius = (min_radius + max_radius) * 0.5;
            double check_x = center_x + test_radius * cos(angle);
            double check_y = center_y + test_radius * sin(angle);
            
            if (isObstacle(check_x, check_y)) {
                max_radius = test_radius;
            } else {
                min_radius = test_radius;
                best_radius = test_radius;
            }
        }
        
        if (best_radius > robot_radius_) { // Geçerli yarıçap
            double final_x = center_x + best_radius * cos(angle);
            double final_y = center_y + best_radius * sin(angle);
            detailed_contour.push_back(createPoint(final_x, final_y, 0.0));
        }
    }
    
    return detailed_contour;
}

// Kapsama tamamlık doğrulama
bool FullCoveragePlanner::verifyCoverageCompleteness(const std::vector<geometry_msgs::PoseStamped>& plan,
                                                    const std::vector<geometry_msgs::Point>& polygon) {
    
    if (plan.empty()) return false;
    
    // Basit doğrulama: robot yarıçapı içindeki alanların kapsanıp kapsanmadığını kontrol et
    double coverage_radius = robot_radius_ * 0.8; // Biraz tolerans
    
    // Poligon sınırları
    double min_x = polygon[0].x, max_x = polygon[0].x;
    double min_y = polygon[0].y, max_y = polygon[0].y;
    
    for (const auto& point : polygon) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    
    // Grid üzerinde kapsama kontrolü
    double resolution = costmap_->getResolution() * 2.0;
    int uncovered_count = 0;
    int total_count = 0;
    
    for (double x = min_x; x <= max_x; x += resolution) {
        for (double y = min_y; y <= max_y; y += resolution) {
            geometry_msgs::Point test_point = createPoint(x, y, 0.0);
            
            if (isPointInPolygon(test_point, polygon) && !isObstacle(x, y)) {
                total_count++;
                
                // Bu nokta planla kapsanıyor mu?
                bool covered = false;
                for (const auto& pose : plan) {
                    if (euclideanDistance(test_point, pose.pose.position) <= coverage_radius) {
                        covered = true;
                        break;
                    }
                }
                
                if (!covered) {
                    uncovered_count++;
                }
            }
        }
    }
    
    double coverage_ratio = total_count > 0 ? 1.0 - (double)uncovered_count / total_count : 0.0;
    ROS_INFO("[FullCoveragePlanner] Coverage completeness: %.1f%% (%d/%d points covered)", 
             coverage_ratio * 100.0, total_count - uncovered_count, total_count);
    
    return coverage_ratio >= 0.95; // %95 kapsama yeterli
}

// Kapsanmamış alanları bul
std::vector<geometry_msgs::Point> FullCoveragePlanner::findUncoveredAreas(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const std::vector<geometry_msgs::Point>& polygon) {
    
    std::vector<geometry_msgs::Point> uncovered_areas;
    
    if (plan.empty()) return uncovered_areas;
    
    double coverage_radius = robot_radius_ * 0.8;
    
    // Poligon sınırları
    double min_x = polygon[0].x, max_x = polygon[0].x;
    double min_y = polygon[0].y, max_y = polygon[0].y;
    
    for (const auto& point : polygon) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    
    // Grid taraması
    double resolution = costmap_->getResolution() * 3.0;
    
    for (double x = min_x; x <= max_x; x += resolution) {
        for (double y = min_y; y <= max_y; y += resolution) {
            geometry_msgs::Point test_point = createPoint(x, y, 0.0);
            
            if (isPointInPolygon(test_point, polygon) && !isObstacle(x, y)) {
                
                // Bu nokta kapsanıyor mu?
                bool covered = false;
                for (const auto& pose : plan) {
                    if (euclideanDistance(test_point, pose.pose.position) <= coverage_radius) {
                        covered = true;
                        break;
                    }
                }
                
                if (!covered) {
                    uncovered_areas.push_back(test_point);
                }
            }
        }
    }
    
    return uncovered_areas;
}

// Diğer fonksiyonların implementasyonu önceki kodla aynı kalacak...
// (isObstacle, isObstacleWithMargin, isPointSafe, createPoint, vb.)

bool FullCoveragePlanner::isObstacle(double world_x, double world_y) {
    if (!costmap_) return true;
    
    unsigned int map_x, map_y;
    if (!costmap_->worldToMap(world_x, world_y, map_x, map_y)) {
        return true;
    }
    
    unsigned char cost = costmap_->getCost(map_x, map_y);
    return (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost != costmap_2d::NO_INFORMATION);
}

bool FullCoveragePlanner::isObstacleWithMargin(double world_x, double world_y, double margin) {
    if (!costmap_) return true;
    
    if (isObstacle(world_x, world_y)) {
        return true;
    }
    
    if (margin <= 0) {
        margin = safety_margin_;
    }
    
    int num_checks = 16;
    double angle_step = 2.0 * M_PI / num_checks;
    
    for (int i = 0; i < num_checks; ++i) {
        double check_x = world_x + margin * cos(i * angle_step);
        double check_y = world_y + margin * sin(i * angle_step);
        
        if (isObstacle(check_x, check_y)) {
            return true;
        }
    }
    
    double half_margin = margin * 0.5;
    for (int i = 0; i < num_checks; i += 2) {
        double check_x = world_x + half_margin * cos(i * angle_step);
        double check_y = world_y + half_margin * sin(i * angle_step);
        
        if (isObstacle(check_x, check_y)) {
            return true;
        }
    }
    
    return false;
}

bool FullCoveragePlanner::isPointSafe(double world_x, double world_y, double safety_radius) {
    return !isObstacleWithMargin(world_x, world_y, safety_radius);
}

bool FullCoveragePlanner::isPointInPolygon(const geometry_msgs::Point& p, 
                                          const std::vector<geometry_msgs::Point>& poly_points_geom) {
    int i, j;
    bool c = false;
    int nvert = poly_points_geom.size();
    if (nvert < 3) return false;
    
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((poly_points_geom[i].y > p.y) != (poly_points_geom[j].y > p.y)) &&
            (poly_points_geom[j].y != poly_points_geom[i].y) && 
            (p.x < (poly_points_geom[j].x - poly_points_geom[i].x) * (p.y - poly_points_geom[i].y) / 
             (poly_points_geom[j].y - poly_points_geom[i].y) + poly_points_geom[i].x)) {
            c = !c;
        }
    }
    return c;
}

geometry_msgs::Point FullCoveragePlanner::createPoint(double x, double y, double z) {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::PoseStamped FullCoveragePlanner::createPoseStamped(
    const geometry_msgs::Point& point, double yaw) {
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.header.stamp = ros::Time::now();
    pose.pose.position = point;
    pose.pose.position.z = 0.01;
    
    tf2::Quaternion q_tf;
    q_tf.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q_tf);
    
    return pose;
}

double FullCoveragePlanner::euclideanDistance(const geometry_msgs::Point& a, 
                                             const geometry_msgs::Point& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

std::string FullCoveragePlanner::pointToString(double x, double y) {
    return std::to_string(static_cast<int>(x * 1000)) + "," + std::to_string(static_cast<int>(y * 1000));
}

// Ana boustrophedon path generation fonksiyonu önceki kodla aynı kalacak
void FullCoveragePlanner::generateBoustrophedonPath(
    const std::vector<geometry_msgs::Point>& poly_corners_geom,
    const geometry_msgs::PoseStamped& start_pose_unused,
    std::vector<geometry_msgs::PoseStamped>& plan) {
    // Önceki implementasyon aynı kalacak - sadece sonunda engel kapsama kontrolü eklenecek
    // ... (önceki kod)
    
    if (poly_corners_geom.size() != 4) { 
        ROS_ERROR("[FullCoveragePlanner::generateBoustrophedonPath] Polygon must have exactly 4 corners."); 
        return; 
    }
    
    if (robot_radius_ <= 0 || sweep_spacing_factor_ <= 0 || path_point_distance_ <= 0) { 
        ROS_ERROR("[FullCoveragePlanner::generateBoustrophedonPath] Invalid parameters"); 
        return; 
    }
    
    if (!costmap_) { 
        ROS_ERROR("[FullCoveragePlanner::generateBoustrophedonPath] Costmap unavailable!"); 
        return; 
    }
    
    // Calculate polygon bounds
    double min_x_poly = poly_corners_geom[0].x, max_x_poly = poly_corners_geom[0].x;
    double min_y_poly = poly_corners_geom[0].y, max_y_poly = poly_corners_geom[0].y;
    
    for (const auto& corner : poly_corners_geom) {
        min_x_poly = std::min(min_x_poly, corner.x);
        max_x_poly = std::max(max_x_poly, corner.x);
        min_y_poly = std::min(min_y_poly, corner.y);
        max_y_poly = std::max(max_y_poly, corner.y);
    }

    // Apply enhanced safety margins
    double effective_robot_size = robot_radius_ + safety_margin_;
    double safe_min_x = min_x_poly + effective_robot_size;
    double safe_max_x = max_x_poly - effective_robot_size;
    double safe_min_y = min_y_poly + effective_robot_size;
    double safe_max_y = max_y_poly - effective_robot_size;
    
    if (safe_min_x >= safe_max_x || safe_min_y >= safe_max_y) {
        ROS_ERROR("[FullCoveragePlanner] Polygon too small for robot with safety margins!");
        return;
    }

    bool current_sweep_left_to_right = true;
    double yaw_l_to_r = 0.0;
    double yaw_r_to_l = M_PI;
    
    geometry_msgs::PoseStamped last_valid_pose_on_plan;
    bool plan_already_has_points = false;
    
    int sweep_count = 0;

    ROS_INFO("[FullCoveragePlanner] Starting COMPREHENSIVE coverage with obstacle avoidance");

    // Ana süpürme döngüsü - önceki implementasyon ile aynı mantık
    for (double y_sweep_center = safe_min_y; y_sweep_center <= safe_max_y;) {
        
        sweep_count++;
        
        double line_separation = calculateAdaptiveSpacing(y_sweep_center, poly_corners_geom);
        
        if (costmap_) {
            double min_sep = costmap_->getResolution() * 3.0;
            line_separation = std::max(line_separation, min_sep);
        }
        
        std::vector<double> x_intersections_on_line;
        
        for (size_t i = 0; i < poly_corners_geom.size(); ++i) {
            const auto& p1 = poly_corners_geom[i];
            const auto& p2 = poly_corners_geom[(i + 1) % poly_corners_geom.size()];
            
            if (std::abs(p1.y - p2.y) > 1e-6) {
                if ((y_sweep_center >= std::min(p1.y, p2.y) - 1e-6) && 
                    (y_sweep_center <= std::max(p1.y, p2.y) + 1e-6)) {
                    double intersect_x = p1.x + (p2.x - p1.x) * (y_sweep_center - p1.y) / (p2.y - p1.y);
                    if (intersect_x >= min_x_poly - effective_robot_size && 
                        intersect_x <= max_x_poly + effective_robot_size) {
                         x_intersections_on_line.push_back(intersect_x);
                    }
                }
            }
        }
        
        std::sort(x_intersections_on_line.begin(), x_intersections_on_line.end());
        x_intersections_on_line.erase(std::unique(x_intersections_on_line.begin(), x_intersections_on_line.end(), 
                                     [](double a, double b){ return std::abs(a-b) < 1e-5; }), 
                                     x_intersections_on_line.end());

        double current_sweep_yaw = current_sweep_left_to_right ? yaw_l_to_r : yaw_r_to_l;

        // Segment işleme - engeller etrafında boşluk bırakır
        for (size_t k = 0; k + 1 < x_intersections_on_line.size(); k += 2) {
            double x_raw_start = x_intersections_on_line[k];
            double x_raw_end = x_intersections_on_line[k+1];

            geometry_msgs::Point mid_p_seg_test = createPoint(
                (x_raw_start + x_raw_end) / 2.0, y_sweep_center, 0.0);
            
            if (!isPointInPolygon(mid_p_seg_test, poly_corners_geom)) {
                continue; 
            }
            
            double x_eff_start = x_raw_start + effective_robot_size;
            double x_eff_end   = x_raw_end   - effective_robot_size;

            if (x_eff_start >= x_eff_end - path_point_distance_ / 2.0) {
                continue;
            }

            double x_iter, x_limit_iter, x_step_iter;
            if (current_sweep_left_to_right) {
                x_iter = x_eff_start; 
                x_limit_iter = x_eff_end; 
                x_step_iter = path_point_distance_;
            } else {
                x_iter = x_eff_end; 
                x_limit_iter = x_eff_start; 
                x_step_iter = -path_point_distance_;
            }

            std::vector<geometry_msgs::PoseStamped> current_sub_segment_plan;
            bool obstructed_on_this_sub_segment = false;
            
            while ((current_sweep_left_to_right ? x_iter <= x_limit_iter + 1e-3 : x_iter >= x_limit_iter - 1e-3)) {
                geometry_msgs::Point current_point_candidate = createPoint(x_iter, y_sweep_center, 0.0);

                if (isPointInPolygon(current_point_candidate, poly_corners_geom) && 
                    isPointSafe(x_iter, y_sweep_center, robot_radius_ + safety_margin_)) {
                    
                    if (obstructed_on_this_sub_segment) { 
                        if (!current_sub_segment_plan.empty()) {
                            plan.insert(plan.end(), current_sub_segment_plan.begin(), current_sub_segment_plan.end());
                            if (!plan.empty()) {
                                last_valid_pose_on_plan = plan.back(); 
                                plan_already_has_points = true;
                            }
                            current_sub_segment_plan.clear();
                        }
                        obstructed_on_this_sub_segment = false; 
                    }

                    geometry_msgs::PoseStamped new_pose;
                    new_pose.pose.position = current_point_candidate;
                    new_pose.pose.position.z = 0.01;
                    tf2::Quaternion q_tf;
                    q_tf.setRPY(0, 0, current_sweep_yaw);
                    new_pose.pose.orientation = tf2::toMsg(q_tf