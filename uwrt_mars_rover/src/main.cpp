#include <rclcpp/rclcpp.hpp>

#include "stdlib.h"

class node : public rclcpp::Node
{
  explicit node(const rclcpp::NodeOptions & options) : Node("node", options) {}
};

int main() { return 0; }
