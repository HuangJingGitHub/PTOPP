#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "../3d/obstacles_3d.hpp"

void VisualizeObstaclesAndPath(const vector<PolygonObstacle3d>& obs_vec, const vector<Point3f>& path) {
    ros::NodeHandle obs_node, path_node;
    ros::Rate update_rate(1);
    ros::Publisher obs_array_pub = obs_node.advertise<visualization_msgs::MarkerArray>("obstacle_marker_array", 10),
                   path_marker_pub = path_node.advertise<visualization_msgs::Marker>("path_marker", 10);
    visualization_msgs::MarkerArray obs_array;
    visualization_msgs::Marker path_marker;
    
    int obs_num = obs_vec.size() - 4, path_node_num = path.size();
    obs_array.markers = vector<visualization_msgs::Marker>(obs_num);
    path_marker.points = vector<geometry_msgs::Point>(path_node_num);
    path_marker.colors = vector<std_msgs::ColorRGBA>(path_node_num);
    string frame_id = "planning_frame";
    // Skip environment walls
    for (int i = 4; i < obs_num + 4; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacle_shapes";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        Point2f obs_centroid(0, 0);
        for (int j = 0; j < 4; j++)
            obs_centroid += obs_vec[i].vertices[j];
        obs_centroid /= 4.0;
        Point2f bottom_side = obs_vec[i].vertices[1] - obs_vec[i].vertices[0];
        float length = cv::norm(obs_vec[i].vertices[1] - obs_vec[i].vertices[0]),
              width = cv::norm(obs_vec[i].vertices[2] - obs_vec[i].vertices[1]),
              angle = atan2(bottom_side.y, bottom_side.x);

        marker.pose.position.x = obs_centroid.x;
        marker.pose.position.y = obs_centroid.y;
        marker.pose.position.z = obs_vec[i].height / 2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = sin(angle / 2);
        marker.pose.orientation.w = cos(angle / 2);

        marker.scale.x = length;
        marker.scale.y = width;
        marker.scale.z = obs_vec[i].height;

        marker.color.r = 0.8275f;
        marker.color.g = 0.8275f;
        marker.color.b = 0.8275f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        obs_array.markers[i - 4] = marker;
    }  

    path_marker.header.frame_id = frame_id;
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "path_points";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;   
    path_marker.pose.orientation.x = 0.0;
    path_marker.pose.orientation.y = 0.0;
    path_marker.pose.orientation.z = 0.0;
    path_marker.pose.orientation.w = 1.0; 
    path_marker.scale.x = 4;
    for (int i = 0; i < path_node_num; i++) {
        geometry_msgs::Point point;
        point.x = path[i].x;
        point.y = path[i].y;
        point.z = path[i].z;
        path_marker.points[i] = point;

        std_msgs::ColorRGBA rgba;
        rgba.r = 0;
        rgba.g = 0;
        rgba.b = 1.0;
        rgba.a = 1.0;
        path_marker.colors[i] = rgba;
    }      

    while (ros::ok()) {
        // Publish the marker
        while (obs_array_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        obs_array_pub.publish(obs_array);
        path_marker_pub.publish(path_marker);
        update_rate.sleep();
    }
}

void VisualizeObstaclesAndPaths(const vector<PolygonObstacle3d>& obs_vec, const vector<vector<Point3f>>& paths) {
    ros::NodeHandle obs_node, path_node;
    ros::Rate update_rate(1);
    ros::Publisher obs_array_pub = obs_node.advertise<visualization_msgs::MarkerArray>("obstacle_marker_array", 10);
    int path_num = paths.size();
    vector<ros::Publisher> path_pubs(path_num);
    for (int i = 0; i < path_num; i++) {
        path_pubs[i] = path_node.advertise<visualization_msgs::Marker>("path_marker_" + to_string(i), 10);
    }

    visualization_msgs::MarkerArray obs_array; 
    int obs_num = obs_vec.size() - 4;
    obs_array.markers = vector<visualization_msgs::Marker>(obs_num);
    string frame_id = "planning_frame";
    // Skip environment walls
    for (int i = 4; i < obs_num + 4; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacle_shapes";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        Point2f obs_centroid(0, 0);
        for (int j = 0; j < 4; j++)
            obs_centroid += obs_vec[i].vertices[j];
        obs_centroid /= 4.0;
        Point2f bottom_side = obs_vec[i].vertices[1] - obs_vec[i].vertices[0];
        float length = cv::norm(obs_vec[i].vertices[1] - obs_vec[i].vertices[0]),
              width = cv::norm(obs_vec[i].vertices[2] - obs_vec[i].vertices[1]),
              angle = atan2(bottom_side.y, bottom_side.x);

        marker.pose.position.x = obs_centroid.x;
        marker.pose.position.y = obs_centroid.y;
        marker.pose.position.z = obs_vec[i].height / 2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = sin(angle / 2);
        marker.pose.orientation.w = cos(angle / 2);

        marker.scale.x = length;
        marker.scale.y = width;
        marker.scale.z = obs_vec[i].height;

        marker.color.r = 0.8275f;
        marker.color.g = 0.8275f;
        marker.color.b = 0.8275f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        obs_array.markers[i - 4] = marker;
    }  

    vector<vector<float>> path_rgbs{{1, 0, 0}, {0, 0.804, 0.424}, {0, 0.604, 0.871}};
    vector<visualization_msgs::Marker> path_markers(path_num);
    for (int i = 0; i < path_num; i++) {
        int path_node_num = paths[i].size();
        path_markers[i].points = vector<geometry_msgs::Point>(path_node_num);
        path_markers[i].colors = vector<std_msgs::ColorRGBA>(path_node_num);
        path_markers[i].header.frame_id = frame_id;
        path_markers[i].header.stamp = ros::Time::now();
        path_markers[i].ns = "path_points";
        path_markers[i].id = i;
        path_markers[i].type = visualization_msgs::Marker::LINE_STRIP;
        path_markers[i].action = visualization_msgs::Marker::ADD;   
        path_markers[i].pose.orientation.x = 0.0;
        path_markers[i].pose.orientation.y = 0.0;
        path_markers[i].pose.orientation.z = 0.0;
        path_markers[i].pose.orientation.w = 1.0; 
        path_markers[i].scale.x = 4;

        for (int j = 0; j < path_node_num; j++) {
            geometry_msgs::Point point;
            point.x = paths[i][j].x;
            point.y = paths[i][j].y;
            point.z = paths[i][j].z;
            path_markers[i].points[j] = point;

            std_msgs::ColorRGBA rgba;
            rgba.r = path_rgbs[i % 3][0];
            rgba.g = path_rgbs[i % 3][1];
            rgba.b = path_rgbs[i % 3][2];
            rgba.a = 1.0;
            path_markers[i].colors[j] = rgba;
        } 
    }

    while (ros::ok()) {
        // Publish the marker
        while (obs_array_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        obs_array_pub.publish(obs_array);
        for (int i = 0; i < path_num; i++) {
            path_pubs[i].publish(path_markers[i]);
        }
        update_rate.sleep();
    }
}

// Visualize solid and dashed path altenately
void VisualizeObstaclesAndPathsInTurn(const vector<PolygonObstacle3d>& obs_vec, const vector<vector<Point3f>>& paths) {
    ros::NodeHandle obs_node, path_node;
    ros::Rate update_rate(1);
    ros::Publisher obs_array_pub = obs_node.advertise<visualization_msgs::MarkerArray>("obstacle_marker_array", 10);
    int path_num = paths.size();
    vector<ros::Publisher> path_pubs(path_num);
    for (int i = 0; i < path_num; i++) {
        path_pubs[i] = path_node.advertise<visualization_msgs::Marker>("path_marker_" + to_string(i), 10);
    }

    visualization_msgs::MarkerArray obs_array; 
    int obs_num = obs_vec.size() - 4;
    obs_array.markers = vector<visualization_msgs::Marker>(obs_num);
    string frame_id = "planning_frame";
    // Skip environment walls
    for (int i = 4; i < obs_num + 4; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacle_shapes";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        Point2f obs_centroid(0, 0);
        for (int j = 0; j < 4; j++)
            obs_centroid += obs_vec[i].vertices[j];
        obs_centroid /= 4.0;
        Point2f bottom_side = obs_vec[i].vertices[1] - obs_vec[i].vertices[0];
        float length = cv::norm(obs_vec[i].vertices[1] - obs_vec[i].vertices[0]),
              width = cv::norm(obs_vec[i].vertices[2] - obs_vec[i].vertices[1]),
              angle = atan2(bottom_side.y, bottom_side.x);

        marker.pose.position.x = obs_centroid.x;
        marker.pose.position.y = obs_centroid.y;
        marker.pose.position.z = obs_vec[i].height / 2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = sin(angle / 2);
        marker.pose.orientation.w = cos(angle / 2);

        marker.scale.x = length;
        marker.scale.y = width;
        marker.scale.z = obs_vec[i].height;

        marker.color.r = 0.8275f;
        marker.color.g = 0.8275f;
        marker.color.b = 0.8275f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        obs_array.markers[i - 4] = marker;
    }  

    vector<vector<float>> path_rgbs{{1, 0, 0}, {0, 0.804, 0.424}, {0, 0.604, 0.871}};
    vector<visualization_msgs::Marker> path_markers(path_num);
    for (int i = 0; i < path_num; i++) {
        int path_node_num = paths[i].size();
        path_markers[i].points = vector<geometry_msgs::Point>(path_node_num);
        path_markers[i].colors = vector<std_msgs::ColorRGBA>(path_node_num);
        path_markers[i].header.frame_id = frame_id;
        path_markers[i].header.stamp = ros::Time::now();
        path_markers[i].ns = "path_points";
        path_markers[i].id = i;
        if (i % 2 == 0)
            path_markers[i].type = visualization_msgs::Marker::LINE_LIST;
        else
            path_markers[i].type = visualization_msgs::Marker::LINE_STRIP;
        path_markers[i].action = visualization_msgs::Marker::ADD;
        path_markers[i].pose.orientation.x = 0.0;
        path_markers[i].pose.orientation.y = 0.0;
        path_markers[i].pose.orientation.z = 0.0;
        path_markers[i].pose.orientation.w = 1.0; 
        path_markers[i].scale.x = 4;

        for (int j = 0; j < path_node_num; j++) {
            geometry_msgs::Point point;
            point.x = paths[i][j].x;
            point.y = paths[i][j].y;
            point.z = paths[i][j].z;
            path_markers[i].points[j] = point;

            std_msgs::ColorRGBA rgba;
            rgba.r = path_rgbs[i / 2 + 1][0];
            rgba.g = path_rgbs[i / 2 + 1][1];
            rgba.b = path_rgbs[i / 2 + 1][2];
            rgba.a = 1.0;
            path_markers[i].colors[j] = rgba;
        }      
        if (i % 2 == 0 && path_node_num % 2 == 1) {
            path_markers[i].points.push_back(path_markers[i].points.back());
            path_markers[i].colors.push_back(path_markers[i].colors.back());
        }
    }

    while (ros::ok()) {
        // Publish the marker
        while (obs_array_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        obs_array_pub.publish(obs_array);
        for (int i = 0; i < path_num; i++) {
            path_pubs[i].publish(path_markers[i]);
        }
        update_rate.sleep();
    }
}