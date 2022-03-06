/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#ifndef GRADIENT_LAYER_HPP_
#define GRADIENT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_util/node_utils.hpp"

#include <map>
#include <vector>
#include <mutex>

#include "rclcpp/create_subscription.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "nav2_dynamic_msgs/msg/obstacle_array.hpp"
#include "nav2_dynamic_msgs/msg/obstacle.hpp"

#include "message_filters/subscriber.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/pose_stamped.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/message_filter.h"

namespace nav2_gradient_costmap_plugin
{

  //! Typedef for a shared dynamic obstacle container
  typedef nav2_dynamic_msgs::msg::ObstacleArray::SharedPtr ObstacleArrayPtr;
  //! Typedef for a shared dynamic obstacle container (read-only access)
  typedef nav2_dynamic_msgs::msg::ObstacleArray::ConstSharedPtr ObstacleArrayConstPtr;

class GradientLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  GradientLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

    /**
   * @brief update the relevant grid cells in the cost map according to the given pose of a person
   *
   * @param obstacle_in_costmap_x map cell x coordinate of obstacle center
   * @param obstacle_in_costmap_y map cell y coordinate of obstacle center
   * @param angle angle of obstacle in map (orientation wrt the global frame)
   * @param costmap costmap to update
   * @param vel velocity module of the obstacle. It is necessary to determine the size of the obstacle space of the person
   */
  void markDynamicObstacle(int obstacle_in_costmap_x, int obstacle_in_costmap_y, 
                                double angle, nav2_costmap_2d::Costmap2D *costmap, 
                                unsigned char * costmap_array, double vel);  
  
  /**
   * @brief calculate value of 2D Gaussian function
   *
   * @param pos_x x coordinate in meters
   * @param pos_y y coordinate in meters
   * @param origin_x x origin of Gaussian
   * @param origin_y y origin of Gaussian
   * @param amplitude amplitude of Gaussian
   * @param variance_x_back variance in x in human back side
   * @param variance_y_back variance in y in human back side
   * @param variance_x_front variance in x in human front side
   * @param variance_y_front variance in y in human front side
   * @param skew skew to zero angle
   * @param p_mot parameter to take the human speed into account
   * @return value of 2D Gaussian function
   */
  double calcGaussian(double pos_x, double pos_y, double origin_x,
                      double origin_y, double amplitude, double variance_x_back,
                      double variance_y_back, double variance_x_front, double variance_y_front,
                      double skew, double p_mot);
  
  /**
   * @brief calculate the distance to the origin of the Gaussian from which the values are lower than the cutoff_value
   *
   * @param cutoff_value Gaussian value bound
   * @param amplitude amplitude of Gaussian
   * @param variance variance
   * @return cutoff distance from origin
   */
  double calcCutoffRadius(double cutoff_value, double amplitude, double variance);


private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_, cost_scaling_factor_;
  nav2_gradient_costmap_plugin::ObstacleArrayConstPtr obstacles;
  std::string obs_frame;   // to store the frame in which obstacles fields are communicated
  std::string global_frame;   // fraame of the costmap: remember global costmap has different frame from local costmap
  std::mutex mutex_;
  std::vector<geometry_msgs::msg::Vector3> tracked_velocities;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Define the params for the Gaussian costs assignment
  double amplitude = 255; ///< amplitude multiplication factor for gaussian 255
  double cutoff_amplitude = 150; //240;//200; //150;//100; //0.1; ///< smallest cost value marked in map
  double variance_x_back = 0.08;//0.3;//0.1;//0.05; ///< x variance for back gaussian function
  double variance_y_back = 0.09;//0.7;//0.2;//0.09;//0.04; ///< y variance for back gaussian function
  double variance_x_front = 0.25;//2.0;//0.7;//0.3;//0.15; ///< x variance for frontal gaussian function
  double variance_y_front =  0.09;//0.7;//0.2;//0.09;//0.04;///< y variance for frontal gaussian function
  double lethal_radius = 0.2; ///< radius around humans with untraversible cost
  double offset_x_ = 0; ///< x offset of Gaussian peak and person center (to the front)   USE SIZE OF THE BOUNDING BOX INSTEAD!
  double offset_y_ =  0;///< y offset of Gaussian peak and person center (to the side)   USE SIZE OF THE BOUNDING BOX INSTEAD!

};

}  // namespace nav2_gradient_costmap_plugin

#endif  // GRADIENT_LAYER_HPP_