#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <vector>

std::vector<geometry_msgs::Point32> corners;
const int MAX_CORNERS = 4;
ros::Publisher polygon_pub;
std::string polygon_frame_id = "map"; // Or get from param

void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (corners.size() < MAX_CORNERS) {
        geometry_msgs::Point32 corner;
        corner.x = msg->point.x;
        corner.y = msg->point.y;
        corner.z = msg->point.z; // Usually 0 for 2D nav
        corners.push_back(corner);
        ROS_INFO("Collected corner %zu/%d at (%.2f, %.2f)", corners.size(), MAX_CORNERS, corner.x, corner.y);

        if (corners.size() == MAX_CORNERS) {
            geometry_msgs::PolygonStamped poly_stamped;
            poly_stamped.header.stamp = ros::Time::now();
            poly_stamped.header.frame_id = msg->header.frame_id; // Use frame from clicked point
            if (!polygon_frame_id.empty() && polygon_frame_id != msg->header.frame_id) {
                 ROS_WARN("Polygon frame_id parameter ('%s') differs from clicked_point frame_id ('%s'). Using clicked_point frame.",
                           polygon_frame_id.c_str(), msg->header.frame_id.c_str());
            } else if (polygon_frame_id.empty()){
                 polygon_frame_id = msg->header.frame_id;
            }
             poly_stamped.header.frame_id = polygon_frame_id;


            for (const auto& p : corners) {
                poly_stamped.polygon.points.push_back(p);
            }
            polygon_pub.publish(poly_stamped);
            ROS_INFO("Published polygon with 4 corners to /coverage_polygon in frame %s. Clearing corners.", poly_stamped.header.frame_id.c_str());
            corners.clear();
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "polygon_publisher_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("polygon_frame_id", polygon_frame_id, "map");

    ros::Subscriber clicked_point_sub = nh.subscribe("/clicked_point", 10, clickedPointCallback);
    polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("/coverage_polygon", 1, true); // Latched publisher

    ROS_INFO("Polygon Publisher Node started. Click 4 points in RVIZ (using Publish Point tool on /clicked_point topic).");
    ros::spin();
    return 0;
}
