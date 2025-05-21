#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys # sys.argv için

def publish_polygon_marker():
    rospy.init_node('polygon_visualizer_node', anonymous=True)
    marker_pub = rospy.Publisher('/coverage_area_marker', Marker, queue_size=1, latch=True)

    # Parametreleri düğümün özel (private) isim alanından almayı dene
    # rosrun ile _param:=value şeklinde verilenler bu şekilde okunur.
    try:
        p1x = rospy.get_param("~p1x")
        p1y = rospy.get_param("~p1y")
        p2x = rospy.get_param("~p2x")
        p2y = rospy.get_param("~p2y")
        p3x = rospy.get_param("~p3x")
        p3y = rospy.get_param("~p3y")
        p4x = rospy.get_param("~p4x")
        p4y = rospy.get_param("~p4y")
        frame_id = rospy.get_param("~frame_id", "map")
    except KeyError as e:
        rospy.logerr("Failed to get private polygon parameter: %s. Node will not publish marker.", str(e))
        rospy.logerr("Ensure parameters are set in the private namespace (e.g. _p1x:=value with rosrun).")
        return

    rospy.loginfo("Polygon Visualizer (params from private ns): P1(%.2f,%.2f), P2(%.2f,%.2f), P3(%.2f,%.2f), P4(%.2f,%.2f) in frame '%s'",
                  p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, frame_id)

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "coverage_polygon_visualization"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    points = []
    points.append(Point(p1x, p1y, 0.01))
    points.append(Point(p2x, p2y, 0.01))
    points.append(Point(p3x, p3y, 0.01))
    points.append(Point(p4x, p4y, 0.01))
    points.append(Point(p1x, p1y, 0.01))
    marker.points = points
    marker.lifetime = rospy.Duration()

    # Latch=True olduğu için periyodik yayına gerek yok, ancak ilk yayının gitmesi için biraz bekleme iyi olabilir.
    rospy.sleep(0.5) 
    marker_pub.publish(marker)
    rospy.loginfo("Coverage area marker published to /coverage_area_marker.")
    
    rospy.spin() # Düğümün kapanmaması ve latch edilmiş mesajın kalıcı olması için

if __name__ == '__main__':
    try:
        publish_polygon_marker()
    except rospy.ROSInterruptException:
        rospy.loginfo("Polygon visualizer shutting down.")
    except Exception as e:
        rospy.logerr("Unhandled error in polygon visualizer: %s", str(e))
