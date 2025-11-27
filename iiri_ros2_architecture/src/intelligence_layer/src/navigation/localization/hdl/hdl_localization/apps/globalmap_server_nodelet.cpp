#include <mutex>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <hdl_localization/srv/set_global_map.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include "extra/resource_tool.hpp"

////***************************start pyc modified***************************/
#include <std_msgs/msg/string.hpp>
using namespace std;
/***************************end pyc modified***************************////


namespace hdl_localization {

class GlobalmapServerNodelet : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNodelet(const rclcpp::NodeOptions& options) : Node("map_server", options)
  {
    initialize_params();

    auto latch_qos = rclcpp::QoS(1).transient_local();
    globalmap_cli_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    globalmap_cli = create_client<hdl_localization::srv::SetGlobalMap>("/get_globalmap", rmw_qos_profile_services_default, globalmap_cli_cb_group);
////***************************start pyc modified***************************/
    map_update_sub = create_subscription<std_msgs::msg::String>(
      "/map_request/pcd",
      latch_qos,
      std::bind(&GlobalmapServerNodelet::map_update_callback, this, std::placeholders::_1)
    );
/***************************end pyc modified***************************////
    globalmap_set_timer = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&GlobalmapServerNodelet::set_global_map_timer_callback, this));
    global_map_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/globalmap", latch_qos);
  }

private:

////***************************start pyc modified***************************/
  void map_update_callback(const std_msgs::msg::String::SharedPtr msg) {
    cout << "in map update callback" << endl;
    RCLCPP_INFO(get_logger(), "Received map request, map path : ");
    std::string globalmap_pcd = msg->data;
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

    // downsample globalmap
    double downsample_resolution = declare_parameter<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
  }
/***************************end pyc modified***************************////


  void initialize_params() {

////***************************start pyc modified***************************/
    cout << endl << "initialize_params" << endl << endl;
/***************************end pyc modified***************************////

    // read globalmap from a pcd file
    // std::string globalmap_pcd = declare_parameter<std::string>("globalmap_pcd", std::string(""));
    this->declare_parameter<std::string>("global_map_name", "map.pcd");
    this->declare_parameter<std::string>("global_map_parent_directory", "map");
    std::string global_map_name = this->get_parameter("global_map_name").as_string();
    std::string global_map_parent_directory = this->get_parameter("global_map_parent_directory").as_string();
    ResourceTool resource_tool;
    std::string global_map_path = resource_tool.get_file_path(global_map_parent_directory, global_map_name);
    if (global_map_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Init map file open failed!");
      return;
    }

////***************************start pyc modified***************************/
    cout << endl << global_map_path << endl << endl;
/***************************end pyc modified***************************////

    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(global_map_path, *globalmap);
    globalmap->header.frame_id = "map";

////***************************start pyc modified***************************/
    // cout << endl << *globalmap << endl << endl;
/***************************end pyc modified***************************////

    bool convert_utm_to_local = declare_parameter<bool>("convert_utm_to_local", true);
    std::ifstream utm_file(global_map_path + ".utm");
    if (utm_file.is_open() && convert_utm_to_local) {
      double utm_easting;
      double utm_northing;
      double altitude;
      utm_file >> utm_easting >> utm_northing >> altitude;
      for(auto& pt : globalmap->points) {
        pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
      }
      RCLCPP_INFO_STREAM(get_logger(), "Global map offset by UTM reference coordinates (x = "
                      << utm_easting << ", y = " << utm_northing << ") and altitude (z = " << altitude << ")");
    }

    // downsample globalmap
    double downsample_resolution = declare_parameter<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
  }

private:
  // ROS
  rclcpp::CallbackGroup::SharedPtr globalmap_cli_cb_group;
  rclcpp::Client<hdl_localization::srv::SetGlobalMap>::SharedPtr globalmap_cli;

  rclcpp::TimerBase::SharedPtr globalmap_set_timer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub;
  pcl::PointCloud<PointT>::Ptr globalmap;

////***************************start pyc modified***************************/
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_update_sub;
/***************************end pyc modified***************************////

  void set_global_map_timer_callback() {
    // sensor_msgs::msg::PointCloud2 globalmap_msg;
    if(!globalmap_cli->wait_for_service(std::chrono::milliseconds(5000))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Node has been shutdown");
        return;
      }
      RCLCPP_WARN(this->get_logger(), "Wait for the HdlLocalization set globalmap service online !");
    }
    sensor_msgs::msg::PointCloud2 globalmap_msg;
    if (!globalmap){
      RCLCPP_ERROR(this->get_logger(), "Global map haven't initialized");
      return;
    }
    pcl::toROSMsg(*globalmap, globalmap_msg);

    auto global_map_request = std::make_shared<hdl_localization::srv::SetGlobalMap::Request>();

    global_map_request->point_cloud_map = globalmap_msg;
    auto globalmap_cli_future = globalmap_cli->async_send_request(global_map_request);

    if (globalmap_cli_future.get()->success) {
      globalmap_set_timer->cancel();
      sensor_msgs::msg::PointCloud2 globalmap_msg;
      pcl::toROSMsg(*globalmap, globalmap_msg);
      global_map_pub->publish(globalmap_msg);
      RCLCPP_INFO(this->get_logger(), "Send global map to HdlLocalizationNodelet success !");
    } else {
      RCLCPP_WARN(this->get_logger(), "Send global map to HdlLocalizationNodelet failed !");
    }
  }
};

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::GlobalmapServerNodelet)

