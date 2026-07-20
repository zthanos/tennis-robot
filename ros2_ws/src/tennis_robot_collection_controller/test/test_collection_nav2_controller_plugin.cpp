#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>
#include <nav2_core/controller.hpp>

TEST(CollectionNav2ControllerPlugin, LoadsViaPluginlib)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader(
    "nav2_core", "nav2_core::Controller");
  auto controller = loader.createSharedInstance(
    "tennis_robot_collection_controller::CollectionNav2Controller");
  EXPECT_NE(controller, nullptr);
}
