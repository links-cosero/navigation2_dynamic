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
#include <visualization_msgs/msg/marker_array.hpp>

#include <ros2_costmap_to_dynamic_obstacles/costmap_to_dynamic_obstalces.h>


class CostmapConversionNode : public rclcpp::Node 
{
public:
  CostmapConversionNode() : Node("costmap_converter_node")
  {
    // Create a pointer to handle the costmap
    costmap_ros_ =
        std::make_shared<nav2_costmap_2d::Costmap2DROS>("converter_costmap");

    costmap_ros_->set_parameter(rclcpp::Parameter("always_send_full_costmap", true)); 
    costmap_ros_->set_parameter(rclcpp::Parameter("publish_frequency", 5.0));
    costmap_ros_->set_parameter(rclcpp::Parameter("resolution", 0.5));
    //costmap_ros_->set_parameter(rclcpp::Parameter("inflation_layer.inflation_radius", 0.30));
    costmap_ros_->declare_parameter( "inflation_layer.inflation_radius", rclcpp::ParameterValue(0.30));
    
    costmap_ros_->declare_parameter( "obstacle_layer.observation_sources", rclcpp::ParameterValue(std::string("scan")));
    costmap_ros_->declare_parameter( "obstacle_layer.scan.clearing", rclcpp::ParameterValue(true)); 
    costmap_ros_->declare_parameter( "obstacle_layer.scan.data_type", rclcpp::ParameterValue(std::string("LaserScan"))); 
    costmap_ros_->declare_parameter( "obstacle_layer.scan.expected_update_rate", rclcpp::ParameterValue(0.0));
    costmap_ros_->declare_parameter( "obstacle_layer.scan.inf_is_valid", rclcpp::ParameterValue(false)); 
    costmap_ros_->declare_parameter( "obstacle_layer.scan.marking", rclcpp::ParameterValue(true)); 
    costmap_ros_->declare_parameter( "obstacle_layer.scan.max_obstacle_height", rclcpp::ParameterValue(2.0));
    costmap_ros_->declare_parameter( "obstacle_layer.scan.min_obstacle_height", rclcpp::ParameterValue(0.0));
    costmap_ros_->declare_parameter( "obstacle_layer.scan.observation_persistence", rclcpp::ParameterValue(0.0));
    costmap_ros_->declare_parameter( "obstacle_layer.scan.obstacle_range", rclcpp::ParameterValue(2.5));
    costmap_ros_->declare_parameter( "obstacle_layer.scan.raytrace_range", rclcpp::ParameterValue(3.0));
    costmap_ros_->declare_parameter( "obstacle_layer.scan.sensor_frame", rclcpp::ParameterValue(std::string("")));
    costmap_ros_->declare_parameter( "obstacle_layer.scan.topic", rclcpp::ParameterValue(std::string("/scan"))); 
    
    costmap_thread_ = std::make_unique<std::thread>(
        [](rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
          rclcpp::spin(node->get_node_base_interface());
        },
        costmap_ros_);
    rclcpp_lifecycle::State state;
    costmap_ros_->on_configure(state);
    costmap_ros_->on_activate(state);

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


    // Set the costmap and translate it to an openCV object
    converter_ = std::make_shared<my_costmap_converter::CostmapToDynamicObstacles>();
    if (converter_) {
      RCLCPP_INFO(get_logger(), "Created converter_ CostmapToDynamicObstacles class");
      param_node_ = std::make_shared<rclcpp::Node>("intra_node", "my_costmap_converter");  // node for allowing dynamic reconfigure of parameters in rqt
      converter_->initialize(param_node_);
      RCLCPP_INFO(get_logger(), "Costmap initialization completed");
    }
    else{
      RCLCPP_INFO(get_logger(), "converter_ pointer instantiation UNSUCCESSFULL");
    }

    // Creating the publisher to the /detection topic
    obstacle_pub_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>(
            detection_topic, 1000);

    // Create timer for publishing on the /detection topic
    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(150),
        std::bind(&CostmapConversionNode::publishCallback, this));

    // Same here for the polygon marker topic   
    std::string obstacle_marker_topic = "costmap_polygon_markers";
    declare_parameter("obstacle_marker_topic",
                      rclcpp::ParameterValue(obstacle_marker_topic));
    get_parameter_or<std::string>("obstacle_marker_topic", obstacle_marker_topic,
                                  obstacle_marker_topic); 

    // Change according to the message type!!
    // marker_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
    //     obstacle_marker_topic, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        obstacle_marker_topic, 10);

  }

  // Callback to publish on the /detection topic
  void publishCallback() {
    converter_->setCostmap2D(costmap_ros_->getCostmap());
    RCLCPP_INFO(get_logger(), "Costmap has been set");
    converter_->compute();
    RCLCPP_INFO(get_logger(), "Costmap conversion completed");

    obstacles =
         converter_->getObstacles();  // HERE PUBLISH THE MESSAGE AS DEFINED IN nav2_dynamic_msgs 

    if (!obstacles) return;

    obstacle_pub_->publish(*obstacles);
    RCLCPP_INFO(get_logger(), "Detected obstacles: %0.0f", (double) obstacles->obstacles.size());

    frame_id_ = costmap_ros_->getGlobalFrameID();

    publishAsMarker(frame_id_, *obstacles);
  }

  std::shared_ptr<rclcpp::Node> getParamNodeHandle() {
    return param_node_;
  }

  // Publish the costmap_converter results on rviz as boxes bounding the detected obstacles
  void publishAsMarker(
      const std::string &frame_id,
      const nav2_dynamic_msgs::msg::ObstacleArray &obstacles) {

    /************************ DISPLAY AS POINT STAMPED **************************/
    // std::vector<geometry_msgs::msg::PointStamped> center_obs;
    // for (auto obstacle : obstacles.obstacles) {
    //   center_obs.emplace_back();
    //   center_obs.back().header.frame_id = frame_id;
    //   center_obs.back().header.stamp = now();
    //   center_obs.back().point = obstacle.position;
    //   marker_pub_->publish(center_obs.back());
    // }

    /************************ DISPLAY AS LINE LIST **************************/
    // if (!obstacles.obstacles.empty()) {
    //   visualization_msgs::msg::Marker line_list;
    //   line_list.header.frame_id = frame_id;
    //   line_list.header.stamp = now();
    //   line_list.ns = "Obstacles Markers";
    //   line_list.action = visualization_msgs::msg::Marker::ADD;
    //   line_list.pose.orientation.w = 1.0;

    //   line_list.id = 0;
    //   line_list.type = visualization_msgs::msg::Marker::LINE_STRIP;

    //   line_list.scale.x = 1.0;
    //   line_list.color.g = 1.0;
    //   line_list.color.a = 1.0;
    //   for (auto obstacle : obstacles.obstacles) {
    //     geometry_msgs::msg::Point point1, point2, point3, point4;
    //     point1.x = obstacle.position.x + (obstacle.size.x / 2);
    //     point1.y = obstacle.position.y + (obstacle.size.y / 2);
    //     line_list.points.push_back(point1);
    //     point2.x = point1.x;
    //     point2.y = point1.y - obstacle.size.y;
    //     line_list.points.push_back(point2);
    //     point3.x = point1.x - obstacle.size.x;
    //     point3.y = point2.y;
    //     line_list.points.push_back(point3);
    //     point4.x = point3.x;
    //     point4.y = point1.y;
    //     line_list.points.push_back(point4);

    //     marker_pub_->publish(line_list);
    //     line_list.id++;
    //   }
    // }
    //   else{
    //     RCLCPP_WARN(get_logger(), "'Obstacles' results empty.");
    //   }

    /*********************** DISPLAY BOUNDING BOXES ***********************/
    visualization_msgs::msg::MarkerArray box;
    //int id_count = 0;
    //box.id = id_count;
    for (auto obstacle : obstacles.obstacles) {
      box.markers.emplace_back();
      box.markers.back().header.frame_id = frame_id;
      box.markers.back().header.stamp = now();
      //box.markers.back().lifetime = rclcpp::Duration(200*1000);  // 200 millisec
      box.markers.back().ns = "Obstacles Markers";
      box.markers.back().type = visualization_msgs::msg::Marker::CUBE;
      box.markers.back().action = visualization_msgs::msg::Marker::ADD;
      box.markers.back().pose.orientation.w = 1.0;
      box.markers.back().pose.position = obstacle.position;
      box.markers.back().scale = obstacle.size;
      box.markers.back().scale.z = 0.01;

      box.markers.back().color.r = 0.0;
      box.markers.back().color.g = 1.0;
      box.markers.back().color.b = 0.0;
      box.markers.back().color.a = 0.5;
      //id_count++;
    }
    marker_pub_->publish(box);
  }

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
  std::shared_ptr<rclcpp::Node> param_node_;
  std::shared_ptr<my_costmap_converter::CostmapToDynamicObstacles> converter_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<std::thread> costmap_thread_;
  rclcpp::Publisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
  my_costmap_converter::ObstacleArrayConstPtr obstacles;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  //rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;


  std::string frame_id_;
  int occupied_min_value_;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto convert_process =
      std::make_shared<CostmapConversionNode>();

  //rclcpp::spin(convert_process);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(convert_process);
  exec.add_node(convert_process->getParamNodeHandle());
  exec.spin();

  return 0;
}
