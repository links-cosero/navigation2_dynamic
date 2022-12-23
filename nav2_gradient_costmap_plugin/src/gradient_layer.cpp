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
#include "nav2_gradient_costmap_plugin/gradient_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_gradient_costmap_plugin
{

GradientLayer::GradientLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max()),
  cost_scaling_factor_(0)
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
GradientLayer::onInitialize()
{
  //auto node = weak_node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("cost_scaling_factor", rclcpp::ParameterValue(10.0));

  auto node = node_.lock();
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "cost_scaling_factor", cost_scaling_factor_);

  need_recalculation_ = false;
  current_ = true;
}


// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
GradientLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
GradientLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  if(layered_costmap_->getDynamicObstacles()==nullptr) {
    return;
  }

  //std::cout << "[DEBUG]: Operative costs." << std::endl;

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap(); // use costmap_ pointer
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
  ObstacleArrayConstPtr obstacles(new nav2_dynamic_msgs::msg::ObstacleArray);

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Mark dynamic obstacles - obstacles are computed in the global frame "/map", thay have to be transformed 
  // into the frame of the map for which the plugin is loaded. local_costmap has "/odom" as global frame.
  // Here we check for trasnform before processing the obstacles
  geometry_msgs::msg::TransformStamped map_to_odom_transform;
  obstacles = layered_costmap_->getDynamicObstacles();
  obs_frame = obstacles->header.frame_id;
  global_frame = layered_costmap_->getGlobalFrameID();
  //obs_frame = obs_frame.substr(1);

  try
  {
    map_to_odom_transform = tf_->lookupTransform(global_frame, obs_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "GradientLayer: Could not transform %s to %s: %s",
            obs_frame.c_str(), global_frame.c_str(), ex.what());
          return;
  }
    

  // Loop through all the obstacles to mark them - first transform data into the current map global frame
  // (local_costmap has "/odom" as global frame)
  for(std::size_t i = 0; i < obstacles->obstacles.size(); i++){

    //loop through all predicted people in the time interval to mark them in the cost map layer
    // transform position
    geometry_msgs::msg::PointStamped obstacle_pos_from_msg;
    obstacle_pos_from_msg.header = obstacles->obstacles.at(i).header;
    obstacle_pos_from_msg.point = obstacles->obstacles.at(i).position;
    geometry_msgs::msg::PointStamped obstacle_pos_stamp;
    tf2::doTransform(obstacle_pos_from_msg, obstacle_pos_stamp, map_to_odom_transform);
    geometry_msgs::msg::Point obstacle_pos = obstacle_pos_stamp.point;
    //geometry_msgs::msg::Point obstacle_pos = obstacles->obstacles.at(i).position;

    // transform velocity
    geometry_msgs::msg::Vector3Stamped obstacle_vel_from_msg;
    obstacle_vel_from_msg.header = obstacles->obstacles.at(i).header;
    obstacle_vel_from_msg.vector = obstacles->obstacles.at(i).velocity;
    geometry_msgs::msg::Vector3Stamped obstacle_vel_stamp;
    tf2::doTransform(obstacle_vel_from_msg,obstacle_vel_stamp,map_to_odom_transform);
    geometry_msgs::msg::Vector3 obstacle_vel = obstacle_vel_stamp.vector;
    //geometry_msgs::msg::Vector3 obstacle_vel = obstacles->obstacles.at(i).velocity;

    // Transform size
    // geometry_msgs::msg::Vector3Stamped obstacle_size_from_msg;
    // obstacle_size_from_msg.header = obstacles->obstacles.at(i).header;
    // obstacle_size_from_msg.vector = obstacles->obstacles.at(i).size;
    // geometry_msgs::msg::Vector3Stamped obstacle_size_stamp;
    // tf2::doTransform(obstacle_vel_from_msg, obstacle_size_stamp, map_to_odom_transform);
    // geometry_msgs::msg::Vector3 obstacle_size = obstacle_size_stamp.vector;
    

    // calculate the angle of the obstacle direction of movement wrt global frame
    double obstacle_angle = -atan2(obstacle_vel.y, obstacle_vel.x);
    // tf2::Quaternion q;
    // tf2::fromMsg((map_to_odom_transform.transform.rotation), q); //getAngle()
    // tf2::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    // double obstacle_angle = yaw + M_PI/2;


    // Calculate the module of the velocity of the obstacle: it is necessary to determine the size of the gaussian function. Assuming that the obstacle is moving
    // at constant velocity.
    double vel = hypot(obstacle_vel.x, obstacle_vel.y);

    //get the position of the obstacle in the map
    int cell_in_costmap_x, cell_in_costmap_y;

    //find cell coordinates of pose in costmap
    master_grid.worldToMapNoBounds(obstacle_pos.x, obstacle_pos.y, cell_in_costmap_x, cell_in_costmap_y);
    
        
    //if position is in the map, mark it in the map (put Gaussian cost values around position
    markDynamicObstacle(cell_in_costmap_x, cell_in_costmap_y, obstacle_angle, &master_grid, master_array, vel);

  }

}


void GradientLayer::markDynamicObstacle(int obstacle_in_costmap_x, int obstacle_in_costmap_y, 
                                double angle, nav2_costmap_2d::Costmap2D *costmap, 
                                unsigned char *costmap_array, double vel)
{
  
  // Parameters to modify the shape and size of the gaussian function
  double max_speed = 2.0;//max_speed_;
  double p_mot = vel/max_speed;

  //clamp the values
  amplitude = amplitude > 0.0 ? amplitude : 0.0;
  amplitude = amplitude < (double) LETHAL_OBSTACLE ? amplitude : (double) LETHAL_OBSTACLE;  // A
  cutoff_amplitude = cutoff_amplitude > 0.0 ? cutoff_amplitude : 0.0;
  //variance_x_back = variance_x_back > 0.0 ? variance_x_back : 0.0;
  //variance_y_back = variance_y_back > 0.0 ? variance_y_back : 0.0;
  //variance_x_front = variance_x_front > 0.0 ? variance_x_front : 0.0;
  //variance_y_front = variance_y_front > 0.0 ? variance_y_front : 0.0;
  lethal_radius = lethal_radius > 0.0 ? lethal_radius : 0.0;

  // Calculate the combination of the 2D gaussian functions around human -----------------------------------
  if(amplitude > 0){

    double resolution = costmap->getResolution();

    //calculate radius around obstacle for cutoff amplitude
    double cutoff_radius_back = std::max(calcCutoffRadius(cutoff_amplitude, amplitude,variance_x_back), calcCutoffRadius(cutoff_amplitude, amplitude,variance_y_back))+0.9;//1.6;//1.2;//0.9; //0.6
    double cutoff_radius_front = std::max(calcCutoffRadius(cutoff_amplitude, amplitude,variance_x_front), calcCutoffRadius(cutoff_amplitude, amplitude,variance_y_front))+1.0;//1.6;//1.2;//1.0;  //0.5


    //calculate corresponding number of grid cells
    int cutoff_radius_grid_back = (int) std::min((cutoff_radius_back / resolution), (double) std::max(costmap->getSizeInCellsY(), costmap->getSizeInCellsX()));
    int cutoff_radius_grid_front = (int) std::min((cutoff_radius_front / resolution), (double) std::max(costmap->getSizeInCellsY(), costmap->getSizeInCellsX()));

    //calculate the origin of the gaussian
    int origin_ix = (obstacle_in_costmap_x + offset_x_ / resolution * cos(angle) + offset_y_ / resolution * -sin(angle));
    int origin_iy = (obstacle_in_costmap_y + offset_x_ / resolution * sin(angle) + offset_y_ / resolution * cos(angle));

    //iterate through relevant grid cells and calculate the amplitude of the gaussian for every cell. Mind costmap contraints
    int x_map_min = std::max(0, origin_ix - cutoff_radius_grid_back);
    int x_map_max = std::min(costmap->getSizeInCellsX(), (unsigned int)(origin_ix + cutoff_radius_grid_front));
    int y_map_min = std::max(0, origin_iy - cutoff_radius_grid_back);
    int y_map_max = std::min(costmap->getSizeInCellsY(), (unsigned int)(origin_iy + cutoff_radius_grid_front));
     

    for(unsigned int i=x_map_min; i< (unsigned int)x_map_max; i++){
      for(unsigned int j=y_map_min; j< (unsigned int)y_map_max; j++){
        
        unsigned char cost;

        //check if the cell is inside the lethal radius around obstacle
        if((hypot(i*resolution - obstacle_in_costmap_x*resolution, j*resolution - obstacle_in_costmap_y*resolution)) < lethal_radius)
          //assign lethal cost
          cost = LETHAL_OBSTACLE;

        //calculate Gaussian value of cell
        else{

          double gauss_ampl = calcGaussian(i*resolution, j*resolution, origin_ix*resolution, origin_iy*resolution, amplitude, variance_x_back, variance_y_back, variance_x_front, variance_y_front, angle, p_mot);
          cost = (unsigned char) gauss_ampl;

          //if the value is outside the cutoff value, set to zero
          if(gauss_ampl < cutoff_amplitude)
            cost = 0;

          //if the cost is larger than the lethal obstacle radius, clamp it
          else if (cost > LETHAL_OBSTACLE)
            cost = LETHAL_OBSTACLE;
        }

        unsigned int index = costmap->getIndex(i, j);
        if(costmap_array[index]<cost) {
          costmap_array[index] = cost;
        }

      }
    }
  }
}


double GradientLayer::calcGaussian(double pos_x, double pos_y, double origin_x,
                                          double origin_y, double amplitude, double variance_x_back,
                                          double variance_y_back, double variance_x_front, double variance_y_front,
                                          double skew, double p_mot)
{

  double dx = pos_x-origin_x;
  double dy = pos_y-origin_y;
  double variance_x;
  double variance_y;

  double distance = sqrt(dx*dx+dy*dy);   // Distanza della cella dall'origine della gaussiana
  double angle = atan2(dy,dx);  // angolo rispetto al centro della gaussiana
  // skew is the angle of the obstacle in the map i.e. the angle of the vel vector wrt the global frame
  double mx = cos(angle-skew) * distance;   // see formula: dcos(theta - theta_h)
  double my = sin(angle-skew) * distance;

  angle += M_PI_4*4;

  if (cos(fabs(skew-angle)) >= 0.0){

    variance_x = variance_x_front;
    variance_x = (1 + p_mot) * variance_x;

    variance_y = variance_y_front;
    variance_y = (1 - p_mot/2) * variance_y;

  } else{
    variance_x = variance_x_back;
    variance_x = (1 - p_mot) * variance_x;
    
    variance_y = variance_y_back;
    variance_y = (1 - p_mot/4) * variance_y;
  }

  double f1 = pow(mx, 2.0)/(2.0 * variance_x);
  double f2 = pow(my, 2.0)/(2.0 * variance_y);

  return amplitude * exp(-(f1 + f2)*cost_scaling_factor_*0.2); //* 0.7;
}

double GradientLayer::calcCutoffRadius(double cutoff_value, double amplitude, double variance){
  return sqrt(-2*variance * log(cutoff_value/amplitude) );
}



}  // namespace nav2_gradient_costmap_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_gradient_costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)