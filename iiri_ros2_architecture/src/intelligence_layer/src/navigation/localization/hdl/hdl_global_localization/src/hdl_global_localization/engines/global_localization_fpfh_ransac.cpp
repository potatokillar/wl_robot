#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>

#include <spdlog/spdlog.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/impl/kdtree.hpp>

#include <hdl_global_localization/ransac/ransac_pose_estimation.hpp>

namespace hdl_global_localization {

GlobalLocalizationEngineFPFH_RANSACParams::GlobalLocalizationEngineFPFH_RANSACParams(std::unique_ptr<rclcpp::Node>& node) {

  node->declare_parameter<bool>("voxel_based", true);
  node->declare_parameter<int>("max_iterations", 1000000);
  node->declare_parameter<int>("matching_budget", 10000);
  node->declare_parameter<int>("correspondence_randomness", 2);
  node->declare_parameter<double>("min_inlier_fraction", 0.25);
  node->declare_parameter<double>("similarity_threshold", 0.5);
  node->declare_parameter<double>("max_correspondence_distance", 1.0);

  node->declare_parameter<double>("normal_estimation_radius", 2.0);
  node->declare_parameter<double>("search_radius", 4.0);

  ransac_params.voxel_based = node->get_parameter("voxel_based").as_bool();
  ransac_params.max_iterations = node->get_parameter("max_iterations").as_int();
  ransac_params.matching_budget = node->get_parameter("matching_budget").as_int();
  ransac_params.correspondence_randomness = node->get_parameter("correspondence_randomness").as_int();
  ransac_params.min_inlier_fraction = node->get_parameter("min_inlier_fraction").as_double();
  ransac_params.similarity_threshold = node->get_parameter("similarity_threshold").as_double();
  ransac_params.max_correspondence_distance = node->get_parameter("max_correspondence_distance").as_double();

  normal_estimation_radius = node->get_parameter("normal_estimation_radius").as_double();
  search_radius = node->get_parameter("search_radius").as_double();
}

GlobalLocalizationEngineFPFH_RANSACParams::~GlobalLocalizationEngineFPFH_RANSACParams() {}

GlobalLocalizationEngineFPFH_RANSAC::GlobalLocalizationEngineFPFH_RANSAC() {
  this->node_ = std::make_unique<rclcpp::Node>("fpfh_ransac_node");
  this->params_ = std::make_unique<GlobalLocalizationEngineFPFH_RANSACParams>(node_);
}

GlobalLocalizationEngineFPFH_RANSAC::~GlobalLocalizationEngineFPFH_RANSAC() {}

pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr GlobalLocalizationEngineFPFH_RANSAC::extract_fpfh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  double normal_estimation_radius = params_->normal_estimation_radius;
  double search_radius = params_->search_radius;

  spdlog::info("Normal Estimation: Radius({})", normal_estimation_radius);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
  nest.setRadiusSearch(normal_estimation_radius);
  nest.setInputCloud(cloud);
  nest.compute(*normals);

  spdlog::info("FPFH Extraction: Search Radius({})", search_radius);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
  fest.setRadiusSearch(search_radius);
  fest.setInputCloud(cloud);
  fest.setInputNormals(normals);
  fest.compute(*features);

  return features;
}

void GlobalLocalizationEngineFPFH_RANSAC::set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  global_map = cloud;
  global_map_features = extract_fpfh(cloud);

  ransac.reset(new RansacPoseEstimation<pcl::FPFHSignature33>(params_->ransac_params));
  ransac->set_target(global_map, global_map_features);
}

GlobalLocalizationResults GlobalLocalizationEngineFPFH_RANSAC::query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) {
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr cloud_features = extract_fpfh(cloud);

  ransac->set_source(cloud, cloud_features);
  auto results = ransac->estimate();

  return results.sort(max_num_candidates);
}

}  // namespace hdl_global_localization