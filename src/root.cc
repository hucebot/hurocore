// Copyright 2025 Ioannis Tsikelis

#include <hurocore/root_node.h>

#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hurocore::RootNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
