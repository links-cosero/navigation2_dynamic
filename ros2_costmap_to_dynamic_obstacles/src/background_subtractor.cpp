/* 
  THIS .cpp IS INSPIRED BY background_subtractor.cpp OF costmap_converter PACKAGE.

  It applies a running average filter to the input costmaps in order to isolate points
  representing dynamic obstacles.
*/


#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cvv/cvv.hpp>
#include <ros2_costmap_to_dynamic_obstacles/background_subtractor.h>

BackgroundSubtractor::BackgroundSubtractor(const Params &parameters): params_(parameters)
{
}

void BackgroundSubtractor::apply(const cv::Mat& image, cv::Mat& fg_mask, int shift_x, int shift_y)
{
  current_frame_ = image;

  // occupancy grids are empty only once in the beginning -> initialize variables
  if (occupancy_grid_fast_.empty() && occupancy_grid_slow_.empty())
  {
    occupancy_grid_fast_ = current_frame_;
    occupancy_grid_slow_ = current_frame_;
    previous_shift_x_ = shift_x;
    previous_shift_y_ = shift_y;
    return;
  }

  // Shift previous occupancy grid to new location (match current_frame_)
  int shift_relative_to_previous_pos_x_ = shift_x - previous_shift_x_;
  int shift_relative_to_previous_pos_y_ = shift_y - previous_shift_y_;
  previous_shift_x_ = shift_x;
  previous_shift_y_ = shift_y;

  // if(shift_relative_to_previous_pos_x_ != 0 || shift_relative_to_previous_pos_y_ != 0)
  transformToCurrentFrame(shift_relative_to_previous_pos_x_, shift_relative_to_previous_pos_y_);

  // cvv::debugFilter(occupancy_grid_fast_, currentFrame_, CVVISUAL_LOCATION);

  // Calculate normalized sum (mean) of nearest neighbors
  int size = 3; // 3, 5, 7, ....
  cv::Mat nearest_neighbor_mean_fast(occupancy_grid_fast_.size(), CV_8UC1);  // Simply initialize a matrix with same size of the image input to apply()
  cv::Mat nearest_neighbor_mean_slow(occupancy_grid_slow_.size(), CV_8UC1);
  cv::boxFilter(occupancy_grid_fast_, nearest_neighbor_mean_fast, -1, cv::Size(size, size), cv::Point(-1, -1), true,  /* see https://docs.opencv.org/4.5.2/d4/d86/group__imgproc__filter.html#gad533230ebf2d42509547d514f7d3fbc3
                                                                                                                          for further info. It blurs the input image with a kernel of size 3x3 */
                cv::BORDER_REPLICATE);
  cv::boxFilter(occupancy_grid_slow_, nearest_neighbor_mean_slow, -1, cv::Size(size, size), cv::Point(-1, -1), true,
                cv::BORDER_REPLICATE);
  //  cv::GaussianBlur(occupancy_grid_fast_, nearest_neighbor_mean_fast, cv::Size(size,size), 1, 1, cv::BORDER_REPLICATE);
  //  cv::GaussianBlur(occupancy_grid_fast_, nearest_neighbor_mean_fast, cv::Size(size,size), 1, 1, cv::BORDER_REPLICATE);

  // compute time mean value for each pixel according to learningrate alpha
  // occupancy_grid_fast_ = beta*(alpha_fast*current_frame_ + (1.0-alpha_fast)*occupancy_grid_fast_) + (1-beta)*nearest_neighbor_mean_fast;
  cv::addWeighted(current_frame_, params_.alpha_fast, occupancy_grid_fast_, (1 - params_.alpha_fast), 0, occupancy_grid_fast_);  // It performs the operations of the previous line using a specific cv function. 
  cv::addWeighted(occupancy_grid_fast_, params_.beta, nearest_neighbor_mean_fast, (1 - params_.beta), 0, occupancy_grid_fast_);  // The final output is occupancy_grid_fast_
  // occupancy_grid_slow_ = beta*(alpha_slow*current_frame_ + (1.0-alpha_slow)*occupancy_grid_slow) + (1-beta)*nearest_neighbor_mean_slow;
  cv::addWeighted(current_frame_, params_.alpha_slow, occupancy_grid_slow_, (1 - params_.alpha_slow), 0, occupancy_grid_slow_);
  cv::addWeighted(occupancy_grid_slow_, params_.beta, nearest_neighbor_mean_slow, (1 - params_.beta), 0, occupancy_grid_slow_);

  // 1) Obstacles should be detected after a minimum response of the fast filter -- we aply subsequent filters to the occupancy_grid_fast_
  //    occupancy_grid_fast_ > min_occupancy_probability
  cv::threshold(occupancy_grid_fast_, occupancy_grid_fast_, params_.min_occupancy_probability, 0 /*unused*/, cv::THRESH_TOZERO);
  // 2) Moving obstacles have a minimum difference between the responses of the slow and fast filter
  //    occupancy_grid_fast_-occupancy_grid_slow_ > min_sep_between_fast_and_slow_filter
  cv::threshold(occupancy_grid_fast_ - occupancy_grid_slow_, fg_mask, params_.min_sep_between_fast_and_slow_filter, 255,
                cv::THRESH_BINARY);
  // 3) Dismiss static obstacles
  //    nearest_neighbor_mean_slow < max_occupancy_neighbors
  cv::threshold(nearest_neighbor_mean_slow, nearest_neighbor_mean_slow, params_.max_occupancy_neighbors, 255, cv::THRESH_BINARY_INV);
  cv::bitwise_and(nearest_neighbor_mean_slow, fg_mask, fg_mask);    // this finally computes the foreground mask fg_mask 

  //visualize("Current frame", currentFrame_);
  cv::Mat setBorderToZero = cv::Mat(current_frame_.size(), CV_8UC1, 0.0);
  int border = 5;
  setBorderToZero(cv::Rect(border, border, current_frame_.cols-2*border, current_frame_.rows-2*border)) = 255;

  cv::bitwise_and(setBorderToZero, fg_mask, fg_mask);

  // cv::imwrite("/home/albers/Desktop/currentFrame.png", currentFrame_);
  // visualize("Foreground mask", fgMask);

  /* Closing Operation: A sequence of erosion and dilation on the binary map results in a closing operation
which reduces noise in the foreground map */
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,  //return an element that can be passed to dilate erode and other morphological transformations available in cv
                                              cv::Size(2*params_.morph_size + 1, 2*params_.morph_size + 1),
                                              cv::Point(params_.morph_size, params_.morph_size));
  
  cv::dilate(fg_mask, fg_mask, element);
  cv::dilate(fg_mask, fg_mask, element);
  cv::erode(fg_mask, fg_mask, element);
}

void BackgroundSubtractor::transformToCurrentFrame(int shift_x, int shift_y)
{
  // TODO: initialize with current_frame (first observed image) rather than zeros

  // Translate (shift) image in costmap coordinates
  // in cv::Mat: shift X -> to the left; shift y -> to the top
  cv::Mat temp_fast, temp_slow;
  cv::Mat translation_mat = (cv::Mat_<double>(2, 3, CV_64F) << 1, 0, -shift_x, 0, 1, -shift_y);
  cv::warpAffine(occupancy_grid_fast_, temp_fast, translation_mat, occupancy_grid_fast_.size()); // can't operate in-place
  cv::warpAffine(occupancy_grid_slow_, temp_slow, translation_mat, occupancy_grid_slow_.size()); // can't operate in-place

  /* see https://docs.opencv.org/3.4/d4/d61/tutorial_warp_affine.html to understand warp affine. It applies
  an affine transformation (rotatio + translation) to the input image. The affine transformation is 
  represented as a 2x3 matrix, "translation_mat" in this case. The method warpAffine applies the transformation:
              warpAffine( input_image, output_image, affine_transf_matrix, warp_dst.size() ); 
   where warp_dst.size() is the desired size of the output image.  */

  // cvv::debugFilter(occupancy_grid_fast_, temp_fast);

  occupancy_grid_fast_ = temp_fast;
  occupancy_grid_slow_ = temp_slow;
}

void BackgroundSubtractor::visualize(const std::string& name, const cv::Mat& image)
{
  if (!image.empty())
  {
    cv::Mat im = image.clone();
    cv::flip(im, im, 0);
    cv::imshow(name, im);
    cv::waitKey(1);
  }
}

void BackgroundSubtractor::updateParameters(const Params &parameters)
{
  params_ = parameters;
}
