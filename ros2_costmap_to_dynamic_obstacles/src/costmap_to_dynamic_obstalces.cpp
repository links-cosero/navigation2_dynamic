/* 
  THIS SOURCE WILL CONTAIN ALL THE METHODS TO EXTRACT 
  THE BLOBS (CLUSTERS) OF DYNAMIC OBSTACLES, TO BE SENT TO THE kf_hungarian_tracker
*/




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

  cv::Mat bg_mat;
  if (publish_static_obstacles_)
  {
    // Get static obstacles
    bg_mat = costmap_mat_ - fg_mask_;
    // visualize("bg_mat", bg_mat);
  }
}