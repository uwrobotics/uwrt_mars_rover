#include <string>
#include "vectornav/vectornav_node.hpp"
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

TEST(vectornav, to_msg_quaternion) {

  rclcpp::NodeOptions options;

  rclcpp::init(0, nullptr);
  vectornav::VectornavNode vnode{options};
  rclcpp::shutdown();

  geometry_msgs::msg::Quaternion quat;
  quat.x = 1;
  quat.y = 2;
  quat.z = 3;
  quat.w = 4;
  vn::math::vec4f vec4 {1, 2, 3, 4};

  EXPECT_EQ(quat, vectornav::toMsg(vec4));
}

TEST(vectornav, to_msg_vector3) {
  rclcpp::NodeOptions options;

  rclcpp::init(0, nullptr);
  vectornav::VectornavNode vnode{options};
  rclcpp::shutdown();

  geometry_msgs::msg::Vector3 tri;
  tri.x = 1;
  tri.y = 2;
  tri.z = 3;
  vn::math::vec3f vec3 {1, 2, 3};

  EXPECT_EQ(tri, vectornav::toMsg(vec3));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
