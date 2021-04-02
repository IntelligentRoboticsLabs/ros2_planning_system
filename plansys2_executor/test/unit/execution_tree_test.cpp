// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>
#include <regex>
#include <iostream>
#include <memory>
#include <thread>
#include <map>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"

#include "rclcpp/rclcpp.hpp"


class BTBuilderTest : public plansys2::BTBuilder
{
public:
  explicit BTBuilderTest(
    rclcpp::Node::SharedPtr node)
  : BTBuilder(node)
  {
  }
};


TEST(executiotest_noden_tree, bt_builder_factory)
{
  auto test_node = rclcpp::Node::make_shared("get_action_from_string");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(test_node);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"robot1", "robot"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"robot2", "robot"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"robot3", "robot"}));

  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"wheels_zone", "zone"}));
  ASSERT_TRUE(
    problem_client->addInstance(
      parser::pddl::tree::Instance{"steering_wheels_zone",
        "zone"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"body_car_zone", "zone"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"assembly_zone", "zone"}));

  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"wheel_1", "piece"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"wheel_2", "piece"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"wheel_3", "piece"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"body_car_1", "piece"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"body_car_2", "piece"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"body_car_3", "piece"}));
  ASSERT_TRUE(
    problem_client->addInstance(
      parser::pddl::tree::Instance{"steering_wheel_1",
        "piece"}));
  ASSERT_TRUE(
    problem_client->addInstance(
      parser::pddl::tree::Instance{"steering_wheel_2",
        "piece"}));
  ASSERT_TRUE(
    problem_client->addInstance(
      parser::pddl::tree::Instance{"steering_wheel_3",
        "piece"}));

  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"car_1", "car"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"car_2", "car"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"car_3", "car"}));

  std::vector<std::string> predicates = {
    "(robot_at robot1 assembly_zone)",
    "(robot_at robot2 assembly_zone)",
    "(robot_at robot3 assembly_zone)",
    "(is_assembly_zone assembly_zone)",
    "(robot_available robot1)",
    "(robot_available robot2)",
    "(robot_available robot3)",
    "(piece_at wheel_1 wheels_zone)",
    "(piece_at body_car_1 body_car_zone)",
    "(piece_at steering_wheel_1 steering_wheels_zone)",
    "(piece_is_wheel wheel_1)",
    "(piece_is_body_car body_car_1)",
    "(piece_is_steering_wheel steering_wheel_1)",
    "(piece_at wheel_2 wheels_zone)",
    "(piece_at body_car_2 body_car_zone)",
    "(piece_at steering_wheel_2 steering_wheels_zone)",
    "(piece_is_wheel wheel_2)",
    "(piece_is_body_car body_car_2)",
    "(piece_is_steering_wheel steering_wheel_2)",
    "(piece_at wheel_3 wheels_zone)",
    "(piece_at body_car_3 body_car_zone)",
    "(piece_at steering_wheel_3 steering_wheels_zone)",
    "(piece_is_wheel wheel_3)",
    "(piece_is_body_car body_car_3)",
    "(piece_is_steering_wheel steering_wheel_3)",
    "(piece_not_used wheel_1)",
    "(piece_not_used wheel_2)",
    "(piece_not_used wheel_3)",
    "(piece_not_used body_car_1)",
    "(piece_not_used body_car_2)",
    "(piece_not_used body_car_3)",
    "(piece_not_used steering_wheel_1)",
    "(piece_not_used steering_wheel_2)",
    "(piece_not_used steering_wheel_3)"};

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(parser::pddl::tree::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      parser::pddl::tree::Goal(
        "(and (car_assembled car_1) (car_assembled car_2) (car_assembled car_3))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();

  for (const auto & action : plan.value()) {
    auto index = action.action + ":" + std::to_string(static_cast<int>(action.time * 1000));

    (*action_map)[index] = plansys2::ActionExecutionInfo();
    (*action_map)[index].durative_action_info =
      get_action_from_string(action.action, domain_client);
    (*action_map)[index].action_executor =
      plansys2::ActionExecutor::make_shared(
      action.action,
      executor_node,
      rclcpp::Duration::from_seconds(action.duration)
      );
  }

  BTBuilderTest exec_tree(test_node);
  auto tree_str = exec_tree.get_tree(plan.value(), action_map);

  std::cout << tree_str << std::endl;
  finish = true;
  t.join();
}


TEST(executiotest_noden_tree, bt_builder_factory_2)
{
  auto test_node = rclcpp::Node::make_shared("get_action_from_string");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(test_node);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"robot1", "robot"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"robot2", "robot"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"robot3", "robot"}));

  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"wheels_zone", "zone"}));
  ASSERT_TRUE(
    problem_client->addInstance(
      parser::pddl::tree::Instance{"steering_wheels_zone",
        "zone"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"body_car_zone", "zone"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"assembly_zone", "zone"}));

  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"wheel_1", "piece"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"wheel_2", "piece"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"wheel_3", "piece"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"body_car_1", "piece"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"body_car_2", "piece"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"body_car_3", "piece"}));
  ASSERT_TRUE(
    problem_client->addInstance(
      parser::pddl::tree::Instance{"steering_wheel_1",
        "piece"}));
  ASSERT_TRUE(
    problem_client->addInstance(
      parser::pddl::tree::Instance{"steering_wheel_2",
        "piece"}));
  ASSERT_TRUE(
    problem_client->addInstance(
      parser::pddl::tree::Instance{"steering_wheel_3",
        "piece"}));

  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"car_1", "car"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"car_2", "car"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"car_3", "car"}));

  std::vector<std::string> predicates = {
    "(robot_at robot1 wheels_zone)",
    "(robot_at robot2 body_car_zone)",
    "(robot_at robot3 steering_wheels_zone)",
    "(is_assembly_zone assembly_zone)",
    "(robot_available robot1)",
    "(robot_available robot2)",
    "(robot_available robot3)",
    "(piece_at wheel_1 wheels_zone)",
    "(piece_at body_car_1 body_car_zone)",
    "(piece_at steering_wheel_1 steering_wheels_zone)",
    "(piece_is_wheel wheel_1)",
    "(piece_is_body_car body_car_1)",
    "(piece_is_steering_wheel steering_wheel_1)",
    "(piece_at wheel_2 wheels_zone)",
    "(piece_at body_car_2 body_car_zone)",
    "(piece_at steering_wheel_2 steering_wheels_zone)",
    "(piece_is_wheel wheel_2)",
    "(piece_is_body_car body_car_2)",
    "(piece_is_steering_wheel steering_wheel_2)",
    "(piece_at wheel_3 wheels_zone)",
    "(piece_at body_car_3 body_car_zone)",
    "(piece_at steering_wheel_3 steering_wheels_zone)",
    "(piece_is_wheel wheel_3)",
    "(piece_is_body_car body_car_3)",
    "(piece_is_steering_wheel steering_wheel_3)",
    "(piece_not_used wheel_1)",
    "(piece_not_used wheel_2)",
    "(piece_not_used wheel_3)",
    "(piece_not_used body_car_1)",
    "(piece_not_used body_car_2)",
    "(piece_not_used body_car_3)",
    "(piece_not_used steering_wheel_1)",
    "(piece_not_used steering_wheel_2)",
    "(piece_not_used steering_wheel_3)"};

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(parser::pddl::tree::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      parser::pddl::tree::Goal(
        std::string("(and (car_assembled car_1) (piece_at body_car_2 assembly_zone)") +
        std::string("(piece_at body_car_3 assembly_zone))"))
  ));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();

  for (const auto & action : plan.value()) {
    auto index = action.action + ":" + std::to_string(static_cast<int>(action.time * 1000));

    (*action_map)[index] = plansys2::ActionExecutionInfo();
    (*action_map)[index].durative_action_info =
      get_action_from_string(action.action, domain_client);
    (*action_map)[index].action_executor =
      plansys2::ActionExecutor::make_shared(
      action.action,
      executor_node,
      rclcpp::Duration::from_seconds(action.duration)
      );
  }

  std::map<std::string, parser::pddl::tree::DurativeAction> durative_actions_map;
  BTBuilderTest exec_tree(test_node);


  // ASSERT_NE(durative_actions_map.find(
  //   "(move robot1 assembly_zone wheels_zone)"),
  //   durative_actions_map.end());

  auto tree_str = exec_tree.get_tree(plan.value(), action_map);

  finish = true;
  t.join();
}

TEST(executiotest_noden_tree, bt_builder_factory_3)
{
  auto test_node = rclcpp::Node::make_shared("get_action_from_string");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(test_node);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"r2d2", "robot"}));
  ASSERT_TRUE(problem_client->addInstance(parser::pddl::tree::Instance{"wp_control", "waypoint"}));
  for (unsigned i = 1; i <= 4; i++) {
    ASSERT_TRUE(
      problem_client->addInstance(
        parser::pddl::tree::Instance{"wp" + std::to_string(i),
          "waypoint"}));
  }

  std::vector<std::string> predicates = {
    "(robot_at r2d2 wp_control)",
    "(charger_at wp3)",
    "(connected wp_control wp1)",
    "(connected wp1 wp_control)",
    "(connected wp_control wp2)",
    "(connected wp2 wp_control)",
    "(connected wp_control wp3)",
    "(connected wp3 wp_control)",
    "(connected wp_control wp4)",
    "(connected wp4 wp_control)"};

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(parser::pddl::tree::Predicate(pred)));
  }

  std::vector<std::string> functions = {
    "(= (speed r2d2) 3)",
    "(= (max_range r2d2) 75)",
    "(= (state_of_charge r2d2) 99)",
    "(= (distance wp1 wp2) 15)",
    "(= (distance wp1 wp3) 18)",
    "(= (distance wp1 wp4) 20)",
    "(= (distance wp1 wp_control) 23)",
    "(= (distance wp_control wp1) 23)",
    "(= (distance wp4 wp1) 20)",
    "(= (distance wp3 wp1) 18)",
    "(= (distance wp2 wp1) 15)",
    "(= (distance wp2 wp3) 23)",
    "(= (distance wp2 wp4) 18)",
    "(= (distance wp2 wp_control) 15)",
    "(= (distance wp_control wp2) 15)",
    "(= (distance wp4 wp2) 18)",
    "(= (distance wp3 wp2) 23)",
    "(= (distance wp3 wp4) 20)",
    "(= (distance wp3 wp_control) 23)",
    "(= (distance wp_control wp3) 23)",
    "(= (distance wp4 wp3) 20)",
    "(= (distance wp4 wp_control) 20)",
    "(= (distance wp_control wp4) 20)"};

  for (const auto & func : functions) {
    ASSERT_TRUE(problem_client->addFunction(parser::pddl::tree::Function(func)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      parser::pddl::tree::Goal(
        std::string("(and (patrolled wp1) (patrolled wp2) (patrolled wp3) (patrolled wp4))"))));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();

  for (const auto & action : plan.value()) {
    auto index = action.action + ":" + std::to_string(static_cast<int>(action.time * 1000));

    (*action_map)[index] = plansys2::ActionExecutionInfo();
    (*action_map)[index].durative_action_info =
      get_action_from_string(action.action, domain_client);
    (*action_map)[index].action_executor =
      plansys2::ActionExecutor::make_shared(
      action.action,
      executor_node,
      rclcpp::Duration::from_seconds(action.duration)
      );
  }

  BTBuilderTest exec_tree(test_node);

  auto tree_str = exec_tree.get_tree(plan.value(), action_map);

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
