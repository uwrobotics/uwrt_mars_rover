#include <rviz_visual_tools/rviz_visual_tools.h>

#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <random>

// Eigen Data types
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using namespace std;

ofstream XYZ_FILE;

class SubscribeAndPublish
{
public:

  SubscribeAndPublish(ros::NodeHandle &nh)  
          : nh_(nh)
  {
    //Topic you want to publish
    pub_ = nh_.advertise<visualization_msgs::Marker>("/uwrt_arm_workspace_markers", 1, true);

    //Topic you want to subscribe
    sub_ = nh_.subscribe("/uwrt_arm_fwdsim", 1, &SubscribeAndPublish::markerCallback, this);
  }

  void markerCallback(const geometry_msgs::Vector3::ConstPtr& msg)
  {
  // write to xyz file
  XYZ_FILE << msg->x << " "<< msg->y << " " << msg->z << "\n";

  visualization_msgs::Marker points;
  points.header.frame_id = "/world";
  points.id = id_;
  points.type = visualization_msgs::Marker::POINTS;
  points.action = visualization_msgs::Marker::ADD;
  points.ns = "point";
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  points.scale.x = 0.025;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  points.scale.y = 0.025;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  points.color.r = 0.8;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  points.color.g = 0.8;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  points.color.b = 0.8;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  points.color.a = 1;

  // Generate point
  geometry_msgs::Point point;
  point.x = msg->x;
  point.y = msg->y;
  point.z = msg->z;
  points.points.push_back(point);

  pub_.publish(points);
  id_++;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  int id_ = 0;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "workspace_markers");
  ros::NodeHandle nh;

  std::string xyz_filepath;
  nh.getParam("/markers", xyz_filepath);
  XYZ_FILE.open(xyz_filepath);

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish sub_pub(nh);

  ros::spin();

  XYZ_FILE.close();
  return 0;
}