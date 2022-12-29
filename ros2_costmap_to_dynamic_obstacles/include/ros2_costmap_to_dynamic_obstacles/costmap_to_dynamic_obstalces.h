/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following code makes use of the OpenCV library.
 * OpenCV is licensed under the terms of the 3-clause BSD License.
 *
 * Authors: Franz Albers, Christoph Rösmann
 *********************************************************************/

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
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

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

#include "unique_identifier_msgs/msg/uuid.hpp"
// STL
#include <memory>

namespace my_costmap_converter
{

  //! Typedef for a shared dynamic obstacle container
  typedef nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr ObstacleArrayPtr;
  //! Typedef for a shared dynamic obstacle container (read-only access)
  typedef nav2_dynamic_msgs::msg::ObstacleArray::ConstSharedPtr ObstacleArrayConstPtr;

  class CostmapToDynamicObstacles
  {
  public:
    /**
      * @brief Destructor
      */
    virtual ~CostmapToDynamicObstacles(){};

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
   * @brief Get updated data from the previously set Costmap2D
   * @sa setCostmap2D
   */
    virtual void updateCostmap2D();

    /**
   * @brief Get a shared instance of the current obstacle container
   * @remarks If compute() or startWorker() has not been called before, this
   * method returns an empty instance!
   * @return Shared instance of the current obstacle container
   */
    ObstacleArrayConstPtr getObstacles();

    /**
   * @brief Gets the last observed contour of a object and converts it from [px]
   * to [m]
   * @return contour2i vector of Point_t, which represents the last observed contour in [m]
   *             in x,y,z coordinates
   */
    std::vector<cv::Point> getContour(std::vector<cv::Point>& contour);

    /**
   * @brief Thread-safe update of the internal obstacle container (that is
   * shared using getObstacles() from outside this
   * class)
   * @param obstacles Updated obstacle container
   */
    void updateObstacleContainer(ObstacleArrayPtr obstacles);

    /** @brief Create a UniqueID message from a UUID object.
   *
   *  @param uu boost::uuids::uuid object.
   *  @returns uuid_msgs/UniqueID message.
   */
    static inline unique_identifier_msgs::msg::UUID toMsg(boost::uuids::uuid const &uu)
    {
      unique_identifier_msgs::msg::UUID msg;
      std::copy(uu.begin(), uu.end(), msg.uuid.begin());
      return msg;
    }

    /**
    * @brief OpenCV Visualization
    * @param name  Id/name of the opencv window
    * @param image Image to be visualized
    */
    void visualize(const std::string &name, const cv::Mat &image);

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
    rclcpp::Node::SharedPtr nh_;

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
