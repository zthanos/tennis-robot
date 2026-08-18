#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>

TEST(CollectionNav2ControllerPlugin, LoadsViaPluginlib)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader(
    "nav2_core", "nav2_core::Controller");
  auto controller = loader.createSharedInstance(
    "tennis_robot_collection_controller::CollectionNav2Controller");
  EXPECT_NE(controller, nullptr);
}

TEST(CollectionProgressGoalCheckerPlugin, LoadsViaPluginlib)
{
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader(
    "nav2_core", "nav2_core::GoalChecker");
  auto checker = loader.createSharedInstance(
    "tennis_robot_collection_controller::CollectionProgressGoalChecker");
  EXPECT_NE(checker, nullptr);
}
