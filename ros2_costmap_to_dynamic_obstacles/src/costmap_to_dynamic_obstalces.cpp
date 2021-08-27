/* 
  THIS SOURCE WILL CONTAIN ALL THE METHODS TO EXTRACT 
  THE BLOBS (CLUSTERS) OF DYNAMIC OBSTACLES, TO BE SENT TO THE kf_hungarian_tracker
*/

#include <ros2_costmap_to_dynamic_obstacles/costmap_to_dynamic_obstalces.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/imgproc.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace my_costmap_converter
{

void CostmapToDynamicObstacles::initialize(rclcpp::Node::SharedPtr nh)
{

  ego_vel_.x = ego_vel_.y = ego_vel_.z = 0;
  costmap_ = nullptr;

  // We need the odometry from the robot to compensate the ego motion
  odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
              odom_topic_,
              rclcpp::SystemDefaultsQoS(),
              std::bind(&CostmapToDynamicObstacles::odomCallback, this, std::placeholders::_1));

  //////////////////////////////////  Foreground detection parameters  //////////////////////////////////
#pragma region
  BackgroundSubtractor::Params bg_sub_params;

  bg_sub_params.alpha_slow = 0.3;
  nh->get_parameter_or<double>("alpha_slow", bg_sub_params.alpha_slow, bg_sub_params.alpha_slow);

  bg_sub_params.alpha_fast = 0.85;
  nh->get_parameter_or<double>("alpha_fast", bg_sub_params.alpha_fast, bg_sub_params.alpha_fast);

  bg_sub_params.beta = 0.85;
  nh->get_parameter_or<double>("beta", bg_sub_params.beta, bg_sub_params.beta);

  bg_sub_params.min_occupancy_probability = 180;
  nh->get_parameter_or<double>("min_occupancy_probability", bg_sub_params.min_occupancy_probability, bg_sub_params.min_occupancy_probability);

  bg_sub_params.min_sep_between_fast_and_slow_filter = 80;
  nh->get_parameter_or<double>("min_sep_between_slow_and_fast_filter", bg_sub_params.min_sep_between_fast_and_slow_filter, bg_sub_params.min_sep_between_fast_and_slow_filter);

  bg_sub_params.max_occupancy_neighbors = 100;
  nh->get_parameter_or<double>("max_occupancy_neighbors", bg_sub_params.max_occupancy_neighbors, bg_sub_params.max_occupancy_neighbors);

  bg_sub_params.morph_size = 1;
  nh->get_parameter_or<int>("morph_size", bg_sub_params.morph_size, bg_sub_params.morph_size);

  bg_sub_ = std::unique_ptr<BackgroundSubtractor>(new BackgroundSubtractor(bg_sub_params));
#pragma endregion

////////////////////////////  Blob detection parameters  ////////////////////////////
#pragma region
  BlobDetector::Params blob_det_params;

  blob_det_params.filterByColor = true; // actually filterByIntensity, always true
  blob_det_params.blobColor = 255;      // Extract light blobs
  blob_det_params.thresholdStep = 256;  // Input for blob detection is already a binary image
  blob_det_params.minThreshold = 127;
  blob_det_params.maxThreshold = 255;
  blob_det_params.minRepeatability = 1;

  blob_det_params.minDistBetweenBlobs = 10;
  nh->get_parameter_or<float>("min_distance_between_blobs", blob_det_params.minDistBetweenBlobs, blob_det_params.minDistBetweenBlobs);

  blob_det_params.filterByArea = true;
  nh->get_parameter_or<bool>("filter_by_area", blob_det_params.filterByArea, blob_det_params.filterByArea);

  blob_det_params.minArea = 3; // Filter out blobs with less pixels
  nh->get_parameter_or<float>("min_area", blob_det_params.minArea, blob_det_params.minArea);

  blob_det_params.maxArea = 300;
  nh->get_parameter_or<float>("max_area", blob_det_params.maxArea, blob_det_params.maxArea);

  blob_det_params.filterByCircularity = true; // circularity = 4*pi*area/perimeter^2
  nh->get_parameter_or<bool>("filter_by_circularity", blob_det_params.filterByCircularity, blob_det_params.filterByCircularity);

  blob_det_params.minCircularity = 0.2;
  nh->get_parameter_or<float>("min_circularity", blob_det_params.minCircularity, blob_det_params.minCircularity);

  blob_det_params.maxCircularity = 1; // maximal 1 (in case of a circle)
  nh->get_parameter_or<float>("max_circularity", blob_det_params.maxCircularity, blob_det_params.maxCircularity);

  blob_det_params.filterByInertia = true; // Filter blobs based on their elongation
  nh->get_parameter_or<bool>("filter_by_intertia", blob_det_params.filterByInertia, blob_det_params.filterByInertia);

  blob_det_params.minInertiaRatio = 0.2; // minimal 0 (in case of a line)
  nh->get_parameter_or<float>("min_inertia_ratio", blob_det_params.minInertiaRatio, blob_det_params.minInertiaRatio);

  blob_det_params.maxInertiaRatio = 1; // maximal 1 (in case of a circle)
  nh->get_parameter_or<float>("max_intertia_ratio", blob_det_params.maxInertiaRatio, blob_det_params.maxInertiaRatio);

  blob_det_params.filterByConvexity = false; // Area of the Blob / Area of its convex hull
  nh->get_parameter_or<bool>("filter_by_convexity", blob_det_params.filterByConvexity, blob_det_params.filterByConvexity);

  blob_det_params.minConvexity = 0; // minimal 0
  nh->get_parameter_or<float>("min_convexity", blob_det_params.minConvexity, blob_det_params.minConvexity);

  blob_det_params.maxConvexity = 1; // maximal 1
  nh->get_parameter_or<float>("max_convexity", blob_det_params.maxConvexity, blob_det_params.maxConvexity);

  blob_det_ = BlobDetector::create(blob_det_params);
#pragma endregion
  
}

void CostmapToDynamicObstacles::compute()
{
  if (costmap_mat_.empty())
    return;

  /////////////////////////// Foreground detection ////////////////////////////////////
  // Dynamic obstacles are separated from static obstacles
  int origin_x = round(costmap_->getOriginX() / costmap_->getResolution());
  int origin_y = round(costmap_->getOriginY() / costmap_->getResolution());
  // ROS_INFO("Origin x  [m]: %f    Origin_y  [m]: %f", costmap_->getOriginX(), costmap_->getOriginY());
  // ROS_INFO("Origin x [px]: %d \t Origin_y [px]: %d", originX, originY);

  bg_sub_->apply(costmap_mat_, fg_mask_, origin_x, origin_y);           /* At the initial time instant, origin_x and origin_y 
                                                                          coincide with the translation coordinate of the costmap
                                                                          wrt the global /map frame */

  // if no foreground object is detected, no ObstacleMsgs need to be published
  if (fg_mask_.empty())
    return;


  /////////////////////////////// Blob detection /////////////////////////////////////
  // Centers and contours of Blobs are detected
  blob_det_->detect(fg_mask_, keypoints_);  //std::vector<cv::KeyPoint> keypoints_; (in header file) is a vector of cv::KeyPoint data struct https://docs.opencv.org/4.5.2/d2/d29/classcv_1_1KeyPoint.html#details
  std::vector<std::vector<cv::Point>> contours = blob_det_->getContours();  // try to change this getContours() to get the centers and size of blobs


  //////////////////////////// Fill ObstacleContainerPtr /////////////////////////////
  ObstacleArrayPtr obstacles(new nav2_dynamic_msgs::msg::ObstacleArray);
  // header.seq is automatically filled
  obstacles->header.stamp = now();
  obstacles->header.frame_id = "/map"; //Global frame /map  
  
  // Here use OpenCV to generate bounding boxes of the detected polygons
  // This makes the obstacles detection less accurate, but the kf_hungarian_tracker needs rectangular
  // shaped obstalces to work
  std::vector<cv::Rect> boundRect( contours.size() );  // https://docs.opencv.org/4.5.2/d2/d44/classcv_1_1Rect__.html#a6fed06513cedd76652389e38c7b1222e
  for (size_t i = 0; i < contours.size(); ++i)
  {
    // create a bounding box for each obstacle
    boundRect[i] = boundingRect( contours[i] );

    obstacles->obstacles.emplace_back();

    // set obstacle ID (UUID)
    unique_identifier_msgs::msg::UUID obstacle_ID;
    boost::uuids::uuid u;
    obstacle_ID = u;
    obstacles->obstacles.back().uuid = obstacle_ID;

    // convert bounding box to size
    geometry_msgs::msg::Vector3 size;
    size.x = boundRect[i].width;
    size.y = boundRect[i].height;
    size.z = 0;
    obstacles->obstacles.back().size = size;

    // set velocity to zero
    geometry_msgs::msg::Vector3 velocity;
    velocity.x, velocity.y, velocity.z = 0;
    obstacles->obstacles.back().velocity = velocity;

    // set the position (the center location)
    geometry_msgs::msg::Point position;
    position.at(i).x = keypoints_.at(i).pt.x;
    position.at(i).y = keypoints_.at(i).pt.y;
    position.at(i).z = 0; // Currently unused!
    obstacles->obstacles.back().position = velocity;

    // Set confidence to 1 for the moment
    obstacles->obstacles.back().score = 1;
  }

  updateObstacleContainer(obstacles)
}



void CostmapToDynamicObstacles::setCostmap2D(nav2_costmap_2d::Costmap2D* costmap)     
{
  if (!costmap)
    return;

  costmap_ = costmap;

  updateCostmap2D();
}

void CostmapToDynamicObstacles::updateCostmap2D()
{
  if (!costmap_->getMutex())
  {
    RCLCPP_ERROR(getLogger(), "Cannot update costmap since the mutex pointer is null");
    return;
  }
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());

  // Initialize costmapMat_ directly with the unsigned char array of costmap_
  //costmap_mat_ = cv::Mat(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), CV_8UC1,
  //                      costmap_->getCharMap()).clone(); // Deep copy: TODO
  costmap_mat_ = cv::Mat(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), CV_8UC1,
                        costmap_->getCharMap());          // CV_8UC1 is the array type: a 8-bit single channel image. 
}

ObstacleArrayConstPtr CostmapToDynamicObstacles::getObstacles()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return obstacles_;
}

void CostmapToDynamicObstacles::updateObstacleContainer(ObstacleArrayPtr obstacles)
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_ = obstacles;
}

void CostmapToDynamicObstacles::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)     // callback function for the subscriber to the odom topic in the initilize() method
{
  RCLCPP_INFO_ONCE(getLogger(), "CostmapToDynamicObstacles: odom received.");

  tf2::Quaternion pose;
  tf2::fromMsg(msg->pose.pose.orientation, pose);

  tf2::Vector3 twistLinear;
  // tf2::fromMsg(msg->twist.twist.linear, twistLinear); // not available in tf2
  twistLinear.setX(msg->twist.twist.linear.x);
  twistLinear.setY(msg->twist.twist.linear.y);
  twistLinear.setZ(msg->twist.twist.linear.z);


  // velocity of the robot in x, y and z coordinates
  tf2::Vector3 vel = tf2::quatRotate(pose, twistLinear);
  ego_vel_.x = vel.x();
  ego_vel_.y = vel.y();
  ego_vel_.z = vel.z();
}

}