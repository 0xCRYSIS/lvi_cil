#pragma once

#include "rclcpp/rclcpp.hpp"                    // #include <ros/ros.h>
#include "std_msgs/msg/header.hpp"              // #include <std_msgs/Header.h>
#include "std_msgs/msg/float32.hpp"             // #include <std_msgs/Float32.h>
#include "std_msgs/msg/bool.hpp"                // #include <std_msgs/Bool.h>
#include "sensor_msgs/msg/imu.hpp"              // #include <sensor_msgs/Imu.h>
#include "sensor_msgs/msg/point_cloud.hpp"      // #include <sensor_msgs/PointCloud.h>
#include "sensor_msgs/msg/image.hpp"            // #include <sensor_msgs/Image.h>
#include "sensor_msgs/image_encodings.hpp"      // #include <sensor_msgs/image_encodings.h>
#include "nav_msgs/msg/path.hpp"                // #include <nav_msgs/Path.h>
#include "nav_msgs/msg/odometry.hpp"            // #include <nav_msgs/Odometry.h>
#include "geometry_msgs/msg/point_stamped.hpp"  // #include <geometry_msgs/PointStamped.h>
#include "visualization_msgs/msg/marker.hpp"    // #include <visualization_msgs/Marker.h>
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"      // #include <tf/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator.h"
#include "../parameters.h"
#include <fstream>

extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            pub_odometry;
extern rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                pub_path;
extern rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_key_poses;

extern nav_msgs::msg::Path path;

extern int IMAGE_ROW, IMAGE_COL;

void registerPub(std::shared_ptr<rclcpp::Node> n);

tf2::Transform transformConversion(const geometry_msgs::msg::TransformStamped t);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::msg::Header &header, const int &failureId);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubInitialGuess(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);