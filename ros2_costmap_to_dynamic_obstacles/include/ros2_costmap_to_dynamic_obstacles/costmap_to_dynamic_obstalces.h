#ifndef COSTMAP_TO_DYNAMIC_OBSTACLES_H_
#define COSTMAP_TO_DYNAMIC_OBSTACLES_H_

// ROS
//#include <costmap_converter/costmap_converter_interface.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
// From defines.h definition of this type, used for the ego velocity.. Is it correct defining it here?
#pragma once
#include <opencv2/opencv.hpp>

typedef float track_t;
typedef cv::Point3_<track_t> Point_t;
#define Mat_t CV_32FC

// dynamic reconfigure
//#include <costmap_converter/CostmapToDynamicObstaclesConfig.h>
//#include <dynamic_reconfigure/server.h>

// Own includes
#include <ros2_costmap_to_dynamic_obstacles/background_subtractor.h>
#include <ros2_costmap_to_dynamic_obstacles/blob_detector.h>
#include <nav2_dynamic_msgs/msg/obstacle_array.hpp>

// STL
#include <memory>

namespace my_costmap_converter
{

  //! Typedef for a shared dynamic obstacle container
  typedef nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr ObstacleArrayPtr;

  class CostmapToDynamicObstacles
  {
  public:
    /**
      * @brief Destructor
      */
    virtual ~CostmapToDynamicObstacles();

    /**
      * @brief Initialize the plugin
      * @param nh Nodehandle that defines the namespace for parameters
      */
    virtual void initialize(rclcpp::Node::SharedPtr nh);

    /**
      * @brief This method performs the actual work (conversion of the costmap to
      * obstacles)
      */
    virtual void compute();

    /**
   * @brief Pass a pointer to the costmap to the plugin.
   * @sa updateCostmap2D
   * @param costmap Pointer to the costmap2d source
   */
    virtual void setCostmap2D(nav2_costmap_2d::Costmap2D *costmap);

   /**
   * @brief Get a shared instance of the current obstacle container
   * @remarks If compute() or startWorker() has not been called before, this
   * method returns an empty instance!
   * @return Shared instance of the current obstacle container
   */
    ObstacleArrayConstPtr getObstacles();

    /**
   * @brief Thread-safe update of the internal obstacle container (that is
   * shared using getObstacles() from outside this
   * class)
   * @param obstacles Updated obstacle container
   */
    void updateObstacleContainer(ObstacleArrayPtr obstacles);

  private:
    std::mutex mutex_;
    nav2_costmap_2d::Costmap2D *costmap_;
    cv::Mat costmap_mat_;
    ObstacleArrayPtr obstacles_;
    cv::Mat fg_mask_;
    std::unique_ptr<BackgroundSubtractor> bg_sub_;
    cv::Ptr<BlobDetector> blob_det_;
    std::vector<cv::KeyPoint> keypoints_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    Point_t ego_vel_;

    std::string odom_topic_ = "/odom";
    //   bool publish_static_obstacles_ = true;

    /**
        * @brief Callback for the odometry messages of the observing robot.
        *
        * Used to convert the velocity of obstacles to the /map frame.
        * @param msg The Pointer to the nav_msgs::Odometry of the observing robot
        */
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  };

}

#endif /* COSTMAP_TO_DYNAMIC_OBSTACLES_H_ */
