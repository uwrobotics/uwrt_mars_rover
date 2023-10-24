#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <pcl/console/time.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>

using namespace pcl;

using PointT = PointXYZ;
using PointCloudT = PointCloud<PointT>;

ros::Publisher PUB_MEAN, PUB_MAX, PUB_MIN;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "workspace_mesh");
  ros::NodeHandle nh;

  // ==============================
  // INIT
  // ==============================

  std::string mesh_filepath_mean;
  std::string mesh_filepath_max;
  std::string mesh_filepath_min;
  nh.getParam("/mesh_mean", mesh_filepath_mean);
  nh.getParam("/mesh_max", mesh_filepath_max);
  nh.getParam("/mesh_min", mesh_filepath_min);

  PUB_MEAN = nh.advertise<visualization_msgs::Marker>("/uwrt_arm_workspace_mesh_mean", 1, true);
  PUB_MAX = nh.advertise<visualization_msgs::Marker>("/uwrt_arm_workspace_mesh_max", 1, true);
  PUB_MIN = nh.advertise<visualization_msgs::Marker>("/uwrt_arm_workspace_mesh_min", 1, true);

  // ==============================
  // MESH MARKERS
  // ==============================

  // MEAN
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.mesh_resource = "file://" + mesh_filepath_mean;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.b = 0.0;
  marker.color.g = 1.0;
  marker.color.r = 0.0;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  marker.color.a = 0.75;
  marker.lifetime = ros::Duration();
  PUB_MEAN.publish(marker);

  // MAX
  // visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.mesh_resource = "file://" + mesh_filepath_max;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.b = 0.0;
  marker.color.g = 0.0;
  marker.color.r = 1.0;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration();
  PUB_MAX.publish(marker);

  // MIN
  // visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.mesh_resource = "file://" + mesh_filepath_min;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  marker.color.b = 0.5;
  marker.color.g = 0.0;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  marker.color.r = 0.5;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration();
  PUB_MIN.publish(marker);

  // ==============================

  ros::Rate rate(1);
  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();
  }
}