/*
This node should subscribe to the global and local costmap topics and provide a pointer 
to the background_suntractor and blob detector.
Finally it should publish an ObstacleArray message to the f_hungarian_tracker
*/

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <ros2_costmap_to_dynamic_obstacles/costmap_to_dynamic_obstalces.h>
// #include <pluginlib/class_loader.hpp>


class CostmapConversionNode : public rclcpp::Node 
{
public:
  CostmapConversionNode() : Node("costmap_converter_node")
  {
    // Create a pointer to handle the costmap
    costmap_ros_ =
        std::make_shared<nav2_costmap_2d::Costmap2DROS>("converter_costmap");
    costmap_thread_ = std::make_unique<std::thread>(
        [](rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
          rclcpp::spin(node->get_node_base_interface());
        },
        costmap_ros_);
    rclcpp_lifecycle::State state;
    costmap_ros_->on_configure(state);
    costmap_ros_->on_activate(state);

    // Create a pointer to this node
    // n_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    RCLCPP_INFO(get_logger(), "Created pointer to the costmap");
    
    // The kf_hungarian_node subscribes to the /detection topic, from which
    // it reads ObstacleArrayMsg.
    std::string detection_topic = "detection";
    declare_parameter("detection_topic",
                      rclcpp::ParameterValue(detection_topic));
    get_parameter_or<std::string>("detection_topic", detection_topic,
                                  detection_topic);


    // Parameter for??? Is it useful for backgound subtractor?? --> Verify
    occupied_min_value_ = 100;
    declare_parameter("occupied_min_value",
                      rclcpp::ParameterValue(occupied_min_value_));
    get_parameter_or<int>("occupied_min_value", occupied_min_value_,
                          occupied_min_value_);

    std::string odom_topic = "/odom";
    declare_parameter("odom_topic", rclcpp::ParameterValue(odom_topic));
    get_parameter_or<std::string>("odom_topic", odom_topic, odom_topic);

    // Understand if it is necessary..   
    // if (converter_) {
    //   converter_->setOdomTopic(odom_topic);
    //   converter_->initialize(
    //       std::make_shared<rclcpp::Node>("intra_node", "costmap_converter"));
    //   converter_->startWorker(std::make_shared<rclcpp::Rate>(5),
    //                           costmap_ros_->getCostmap(), true);  // startWorker get a pointer to the costmap and defines the rate at which it must be
    //                                                               // be updated.. so maybe we don't need since we don't update the costmap in this package
    // }

    // do conversion here: call to costmap_to_dynamic_obstacles in order to activate
    // background_subtractor and blob_detector
    // converter_ =
    //     std::make_shared<my_costmap_converter::CostmapToDynamicObstacles>;
    
      // Set the costmap and translate it to an openCV object
    converter_->initialize(std::make_shared<rclcpp::Node>("intra_node", "my_costmap_converter"));  // è necessario creare un nodo per il costmap converter?
    converter_->setCostmap2D(costmap_ros_->getCostmap());
    converter_->compute();

    // Creating the publisher to the /detection topic
    // HERE MUST BE CHANGED IN ObstacleArrayMsg DEFINED IN THE nav2_dynamic_msgs
    // FROM THE kf_hungarian_tracker PACKAGE
    obstacle_pub_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>(
            detection_topic, 1000);

    // Create timer for publishing on the /detection topic
    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&CostmapConversionNode::publishCallback, this));
  }

  // Callback to publish on the /detection topic
  void publishCallback() {
    my_costmap_converter::ObstacleArrayConstPtr obstacles =
         converter_->getObstacles();  // HERE PUBLISH THE MESSAGE AS DEFINED IN nav2_dynamic_msgs 

    if (!obstacles) return;

    obstacle_pub_->publish(*obstacles);

    // frame_id_ = costmap_ros_->getGlobalFrameID();

    // publishAsMarker(frame_id_, *obstacles);
  }

  // The following has been commented out, since it should only serve to
  // publish the costmap_converter results on rviz, we only need to publish on the 
  // /detection topic
  #pragma region 
  // void publishAsMarker(
  //     const std::string &frame_id,
  //     const std::vector<geometry_msgs::msg::PolygonStamped> &polygonStamped) {
  //   visualization_msgs::msg::Marker line_list;
  //   line_list.header.frame_id = frame_id;
  //   line_list.header.stamp = now();
  //   line_list.ns = "Polygons";
  //   line_list.action = visualization_msgs::msg::Marker::ADD;
  //   line_list.pose.orientation.w = 1.0;

  //   line_list.id = 0;
  //   line_list.type = visualization_msgs::msg::Marker::LINE_LIST;

  //   line_list.scale.x = 0.1;
  //   line_list.color.g = 1.0;
  //   line_list.color.a = 1.0;

  //   for (std::size_t i = 0; i < polygonStamped.size(); ++i) {
  //     for (int j = 0; j < (int)polygonStamped[i].polygon.points.size() - 1;
  //          ++j) {
  //       geometry_msgs::msg::Point line_start;
  //       line_start.x = polygonStamped[i].polygon.points[j].x;
  //       line_start.y = polygonStamped[i].polygon.points[j].y;
  //       line_list.points.push_back(line_start);
  //       geometry_msgs::msg::Point line_end;
  //       line_end.x = polygonStamped[i].polygon.points[j + 1].x;
  //       line_end.y = polygonStamped[i].polygon.points[j + 1].y;
  //       line_list.points.push_back(line_end);
  //     }
  //     // close loop for current polygon
  //     if (!polygonStamped[i].polygon.points.empty() &&
  //         polygonStamped[i].polygon.points.size() != 2) {
  //       geometry_msgs::msg::Point line_start;
  //       line_start.x = polygonStamped[i].polygon.points.back().x;
  //       line_start.y = polygonStamped[i].polygon.points.back().y;
  //       line_list.points.push_back(line_start);
  //       if (line_list.points.size() % 2 != 0) {
  //         geometry_msgs::msg::Point line_end;
  //         line_end.x = polygonStamped[i].polygon.points.front().x;
  //         line_end.y = polygonStamped[i].polygon.points.front().y;
  //         line_list.points.push_back(line_end);
  //       }
  //     }
  //   }
  //   marker_pub_->publish(line_list);
  // }

  // void publishAsMarker(
  //     const std::string &frame_id,
  //     const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles) {
  //   visualization_msgs::msg::Marker line_list;
  //   line_list.header.frame_id = frame_id;
  //   line_list.header.stamp = now();
  //   line_list.ns = "Polygons";
  //   line_list.action = visualization_msgs::msg::Marker::ADD;
  //   line_list.pose.orientation.w = 1.0;

  //   line_list.id = 0;
  //   line_list.type = visualization_msgs::msg::Marker::LINE_LIST;

  //   line_list.scale.x = 0.1;
  //   line_list.color.g = 1.0;
  //   line_list.color.a = 1.0;

  //   for (const auto &obstacle : obstacles.obstacles) {
  //     for (int j = 0; j < (int)obstacle.polygon.points.size() - 1; ++j) {
  //       geometry_msgs::msg::Point line_start;
  //       line_start.x = obstacle.polygon.points[j].x;
  //       line_start.y = obstacle.polygon.points[j].y;
  //       line_list.points.push_back(line_start);
  //       geometry_msgs::msg::Point line_end;
  //       line_end.x = obstacle.polygon.points[j + 1].x;
  //       line_end.y = obstacle.polygon.points[j + 1].y;
  //       line_list.points.push_back(line_end);
  //     }
  //     // close loop for current polygon
  //     if (!obstacle.polygon.points.empty() &&
  //         obstacle.polygon.points.size() != 2) {
  //       geometry_msgs::msg::Point line_start;
  //       line_start.x = obstacle.polygon.points.back().x;
  //       line_start.y = obstacle.polygon.points.back().y;
  //       line_list.points.push_back(line_start);
  //       if (line_list.points.size() % 2 != 0) {
  //         geometry_msgs::msg::Point line_end;
  //         line_end.x = obstacle.polygon.points.front().x;
  //         line_end.y = obstacle.polygon.points.front().y;
  //         line_list.points.push_back(line_end);
  //       }
  //     }
  //   }
  //   marker_pub_->publish(line_list);
  // }
  #pragma endregion


private:
  rclcpp::Node::SharedPtr n_;
  std::shared_ptr<my_costmap_converter::CostmapToDynamicObstacles> converter_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<std::thread> costmap_thread_;
  rclcpp::Publisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
  // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  std::string frame_id_;
  int occupied_min_value_;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto convert_process =
      std::make_shared<CostmapConversionNode>();

  rclcpp::spin(convert_process);

  return 0;
}
