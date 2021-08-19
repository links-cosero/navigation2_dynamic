#ifndef COSTMAP_TO_DYNAMIC_OBSTACLES_H_
#define COSTMAP_TO_DYNAMIC_OBSTACLES_H_

// ROS
//#include <costmap_converter/costmap_converter_interface.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

// dynamic reconfigure
//#include <costmap_converter/CostmapToDynamicObstaclesConfig.h>
//#include <dynamic_reconfigure/server.h>

// Own includes
#include <ros2_costmap_to_dynamic_obstacles/background_subtractor.h>
#include <ros2_costmap_to_dynamic_obstacles/blob_detector.h>

// STL
#include <memory>






#endif /* COSTMAP_TO_DYNAMIC_OBSTACLES_H_ */
