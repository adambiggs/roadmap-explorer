#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>

#include <gtest/gtest.h>
#include "roadmap_explorer/Nav2Interface.hpp"

namespace
{
using namespace std::chrono_literals;
using Navigate = nav2_msgs::action::NavigateToPose;
using ServerGoal = rclcpp_action::ServerGoalHandle<Navigate>;
using Interface = roadmap_explorer::Nav2Interface<Navigate>;
using Status = roadmap_explorer::NavGoalStatus;

class TestInterface : public Interface
{
public:
  using Interface::Interface;
  using Interface::nav2GoalResultCallback;
  using Interface::requestCancel;
};

class NavigationCancel : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(92);
    context->init(0, nullptr, init_options);
    rclcpp::NodeOptions options;
    options.context(context);
    const auto ns = "/navigation_cancel_" + std::to_string(getpid());
    server_node = std::make_shared<rclcpp::Node>("navigator", ns, options);
    client_node = std::make_shared<nav2_util::LifecycleNode>("explorer", ns, options);
    action = rclcpp_action::create_server<Navigate>(
      server_node, "navigate",
      [this](const auto &, auto) {
        ++goal_requests;
        return accept ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
               rclcpp_action::GoalResponse::REJECT;
      },
      [this](std::shared_ptr<ServerGoal> goal) {
        cancelled_goals.push_back(goal->get_goal_id());
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<ServerGoal> goal) { accepted = goal; });
    ParameterHandler::createInstance();
    parameterInstance.setValue<std::string>("explorationBT.nav2_bt_xml", "");
    client = std::make_unique<TestInterface>(client_node, "navigate", "update", "plan");

    rclcpp::ExecutorOptions exec_options;
    exec_options.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(exec_options);
    executor->add_node(client_node->get_node_base_interface());
    executor->add_node(server_node);
  }

  void TearDown() override
  {
    client.reset();
    executor.reset();
    action.reset();
    accepted.reset();
    client_node.reset();
    server_node.reset();
    context->shutdown("test complete");
    ParameterHandler::destroyInstance();
  }

  template<typename Predicate>
  bool spinUntil(Predicate done)
  {
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!done()) {
      auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining <= 0s) {
        return false;
      }
      executor->spin_once(remaining);
    }
    return true;
  }

  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr server_node;
  std::shared_ptr<nav2_util::LifecycleNode> client_node;
  rclcpp_action::Server<Navigate>::SharedPtr action;
  std::shared_ptr<ServerGoal> accepted;
  std::unique_ptr<TestInterface> client;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  bool accept = true;
  int goal_requests = 0;
  std::vector<rclcpp_action::GoalUUID> cancelled_goals;
  geometry_msgs::msg::PoseStamped pose;
};

TEST_F(NavigationCancel, StopBeforeAcceptanceWaitsForChildResult)
{
  ASSERT_TRUE(client->sendGoal(pose));
  // Withhold executor dispatch until the caller has requested the stop.
  client->cancelAllGoals();
  ASSERT_FALSE(client->isGoalTerminated());
  ASSERT_EQ(goal_requests, 0);
  ASSERT_TRUE(spinUntil([this] { return accepted && accepted->is_canceling(); }));
  ASSERT_EQ(cancelled_goals.size(), 1u);
  EXPECT_EQ(cancelled_goals.front(), accepted->get_goal_id());
  EXPECT_FALSE(client->isGoalTerminated());
  client->cancelAllGoals();
  accepted->canceled(std::make_shared<Navigate::Result>());
  ASSERT_TRUE(spinUntil([this] { return client->isGoalTerminated(); }));
  EXPECT_EQ(client->getGoalStatus(), Status::CANCELLED);
  EXPECT_EQ(cancelled_goals.size(), 1u);
}

TEST_F(NavigationCancel, StopAfterAcceptanceOnlyCancelsOwnedGoal)
{
  auto other_client = rclcpp_action::create_client<Navigate>(server_node, "navigate");
  ASSERT_TRUE(other_client->wait_for_action_server(5s));
  auto other_response = other_client->async_send_goal(Navigate::Goal{});
  ASSERT_TRUE(spinUntil([&] { return other_response.wait_for(0s) == std::future_status::ready; }));
  ASSERT_TRUE(other_response.get());
  auto other_goal = accepted;

  ASSERT_TRUE(client->sendGoal(pose));
  ASSERT_TRUE(spinUntil([this] { return client->isGoalOngoing(); }));
  client->cancelAllGoals();
  ASSERT_TRUE(spinUntil([this] { return accepted->is_canceling(); }));
  ASSERT_EQ(cancelled_goals.size(), 1u);
  EXPECT_EQ(cancelled_goals.front(), accepted->get_goal_id());
  EXPECT_TRUE(other_goal->is_executing());
  EXPECT_FALSE(client->isGoalTerminated());
  accepted->canceled(std::make_shared<Navigate::Result>());
  ASSERT_TRUE(spinUntil([this] { return client->isGoalTerminated(); }));
  other_goal->succeed(std::make_shared<Navigate::Result>());
}

TEST_F(NavigationCancel, UnknownResultDoesNotReleaseOrForgetCancellingGoal)
{
  ASSERT_TRUE(client->sendGoal(pose));
  ASSERT_TRUE(spinUntil([this] { return client->isGoalOngoing(); }));
  client->cancelAllGoals();
  Interface::GoalHandle::WrappedResult unknown;
  unknown.goal_id = accepted->get_goal_id();
  unknown.code = rclcpp_action::ResultCode::UNKNOWN;
  client->nav2GoalResultCallback(unknown);
  EXPECT_FALSE(client->isGoalTerminated());
  EXPECT_FALSE(client->isGoalHandleNull());
  ASSERT_TRUE(spinUntil([this] { return accepted->is_canceling(); }));
  accepted->canceled(std::make_shared<Navigate::Result>());
  ASSERT_TRUE(spinUntil([this] { return client->isGoalTerminated(); }));
}

TEST_F(NavigationCancel, SynchronousCancelFailureDoesNotEscapeTheResponseCallback)
{
  auto other_client = rclcpp_action::create_client<Navigate>(server_node, "navigate");
  ASSERT_TRUE(other_client->wait_for_action_server(5s));
  auto response = other_client->async_send_goal(Navigate::Goal{});
  ASSERT_TRUE(spinUntil([&] { return response.wait_for(0s) == std::future_status::ready; }));
  ASSERT_TRUE(response.get());
  // A handle unknown to this client makes rclcpp_action throw during submission.
  EXPECT_NO_THROW(client->requestCancel(response.get()));
  EXPECT_TRUE(accepted->is_executing());
  accepted->succeed(std::make_shared<Navigate::Result>());
}

TEST_F(NavigationCancel, RejectionReleasesPendingStop)
{
  accept = false;
  ASSERT_TRUE(client->sendGoal(pose));
  client->cancelAllGoals();
  ASSERT_FALSE(client->isGoalTerminated());
  ASSERT_TRUE(spinUntil([this] { return client->isGoalTerminated(); }));
  EXPECT_EQ(client->getGoalStatus(), Status::REJECTED);
  EXPECT_EQ(goal_requests, 1);
  EXPECT_TRUE(cancelled_goals.empty());
}

TEST_F(NavigationCancel, ClearPlanReplacesTheLastPlanAViewerSaw)
{
  std::vector<nav_msgs::msg::Path> plans;
  auto viewer = server_node->create_subscription<nav_msgs::msg::Path>(
    "plan", 10, [&](nav_msgs::msg::Path::SharedPtr plan) { plans.push_back(*plan); });
  auto planner = server_node->create_publisher<nav_msgs::msg::Path>("plan", 1);
  ASSERT_TRUE(spinUntil([&] { return viewer->get_publisher_count() == 2; }));
  nav_msgs::msg::Path plan;
  plan.poses.resize(3);
  planner->publish(plan);
  ASSERT_TRUE(spinUntil([&] { return !plans.empty(); }));
  ASSERT_EQ(plans.back().poses.size(), 3u);

  client->clearPlan("map");
  ASSERT_TRUE(spinUntil([&] { return plans.size() == 2; }));
  EXPECT_TRUE(plans.back().poses.empty());
  EXPECT_EQ(plans.back().header.frame_id, "map");
}
}  // namespace
