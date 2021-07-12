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

#include "plansys2_problem_expert/ProblemExpert.hpp"

#include <optional>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>
#include <memory>
#include <set>
#include <map>

#include "plansys2_core/Utils.hpp"
#include "plansys2_pddl_parser/Domain.h"
#include "plansys2_pddl_parser/Instance.h"
#include "plansys2_problem_expert/Utils.hpp"

#include "plansys2_core/Types.hpp"

namespace plansys2
{

ProblemExpert::ProblemExpert(std::shared_ptr<DomainExpert> & domain_expert)
: domain_expert_(domain_expert)
{
}

bool
ProblemExpert::addInstance(const plansys2::Instance & instance)
{
  if (!isValidType(instance.type)) {
    return false;
  } else if (existInstance(instance.name)) {
    return false;
  } else {
    instances_.push_back(instance);
    return true;
  }
}

std::vector<plansys2::Instance>
ProblemExpert::getInstances()
{
  return instances_;
}

bool
ProblemExpert::removeInstance(const plansys2::Instance & instance)
{
  bool found = false;
  int i = 0;

  while (!found && i < instances_.size()) {
    if (instances_[i].name == instance.name) {
      found = true;
      instances_.erase(instances_.begin() + i);
    }
    i++;
  }

  // (fmrico)ToDo: We should remove all goals containing the removed instance
  removeFunctionsReferencing(instance);
  removePredicatesReferencing(instance);

  return found;
}

std::optional<plansys2::Instance>
ProblemExpert::getInstance(const std::string & instance_name)
{
  plansys2::Instance ret;

  bool found = false;
  int i = 0;
  while (i < instances_.size() && !found) {
    if (instances_[i].name == instance_name) {
      found = true;
      ret = instances_[i];
    }
    i++;
  }

  if (found) {
    return ret;
  } else {
    return {};
  }
}

std::vector<plansys2::Predicate>
ProblemExpert::getPredicates()
{
  return predicates_;
}

bool
ProblemExpert::addPredicate(const plansys2::Predicate & predicate)
{
  plansys2::Predicate predicate_lower = predicate;
  std::transform(predicate.name.begin(), predicate.name.end(), predicate_lower.name.begin(),
    [](unsigned char c){ return std::tolower(c);});

  if (!existPredicate(predicate_lower)) {
    if (isValidPredicate(predicate_lower)) {
      predicates_.push_back(predicate_lower);
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

bool
ProblemExpert::removePredicate(const plansys2::Predicate & predicate)
{
  bool found = false;
  int i = 0;

  plansys2::Predicate predicate_lower = predicate;
  std::transform(predicate.name.begin(), predicate.name.end(), predicate_lower.name.begin(),
    [](unsigned char c){ return std::tolower(c);});

  if (!isValidPredicate(predicate_lower)) {  // if predicate is not valid, error
    return false;
  }
  while (!found && i < predicates_.size()) {
    if (parser::pddl::checkNodeEquality(predicates_[i], predicate_lower)) {
      found = true;
      predicates_.erase(predicates_.begin() + i);
    }
    i++;
  }

  return true;
}

std::optional<plansys2::Predicate>
ProblemExpert::getPredicate(const std::string & expr)
{
  std::string expr_lower = expr;
  std::transform(expr.begin(), expr.end(), expr_lower.begin(),
    [](unsigned char c){ return std::tolower(c);});

  plansys2::Predicate ret;
  plansys2::Predicate pred = parser::pddl::fromStringPredicate(expr_lower);

  bool found = false;
  size_t i = 0;
  while (i < predicates_.size() && !found) {
    if (parser::pddl::checkNodeEquality(predicates_[i], pred)) {
      found = true;
      ret = predicates_[i];
    }
    i++;
  }

  if (found) {
    return ret;
  } else {
    return {};
  }
}

std::vector<plansys2::Function>
ProblemExpert::getFunctions()
{
  return functions_;
}

bool
ProblemExpert::addFunction(const plansys2::Function & function)
{
  if (!existFunction(function)) {
    if (isValidFunction(function)) {
      functions_.push_back(function);
      return true;
    } else {
      return false;
    }
  } else {
    return updateFunction(function);
  }
}

bool
ProblemExpert::removeFunction(const plansys2::Function & function)
{
  bool found = false;
  int i = 0;

  if (!isValidFunction(function)) {  // if function is not valid, error
    return false;
  }
  while (!found && i < functions_.size()) {
    if (parser::pddl::checkNodeEquality(functions_[i], function)) {
      found = true;
      functions_.erase(functions_.begin() + i);
    }
    i++;
  }

  return true;
}

bool
ProblemExpert::updateFunction(const plansys2::Function & function)
{
  if (existFunction(function)) {
    if (isValidFunction(function)) {
      removeFunction(function);
      functions_.push_back(function);
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

std::optional<plansys2::Function>
ProblemExpert::getFunction(const std::string & expr)
{
  plansys2::Function ret;
  plansys2::Function func = parser::pddl::fromStringFunction(expr);

  bool found = false;
  size_t i = 0;
  while (i < functions_.size() && !found) {
    if (parser::pddl::checkNodeEquality(functions_[i], func)) {
      found = true;
      ret = functions_[i];
    }
    i++;
  }

  if (found) {
    return ret;
  } else {
    return {};
  }
}

bool
ProblemExpert::removeFunctionsReferencing(const plansys2_msgs::msg::Param & param)
{
  int i = 0;

  while (i < functions_.size()) {
    bool found = false;
    for (plansys2::Instance parameter : functions_[i].parameters) {
      if (parameter.name == param.name) {
        functions_.erase(functions_.begin() + i);
        found = true;
        break;
      }
    }
    if (!found) {
      i++;
    }
  }
  return false;
}

bool
ProblemExpert::removePredicatesReferencing(const plansys2_msgs::msg::Param & param)
{
  int i = 0;

  while (i < predicates_.size()) {
    bool found = false;
    for (plansys2::Instance parameter : predicates_[i].parameters) {
      if (parameter.name == param.name) {
        predicates_.erase(predicates_.begin() + i);
        found = true;
        break;
      }
    }
    if (!found) {
      i++;
    }
  }
  return false;
}

plansys2::Goal
ProblemExpert::getGoal()
{
  return goal_;
}

bool
ProblemExpert::setGoal(const plansys2::Goal & goal)
{
  if (isValidGoal(goal)) {
    goal_ = goal;
    return true;
  } else {
    return false;
  }
}

bool ProblemExpert::isGoalSatisfied(const plansys2::Goal & goal)
{
  return check(goal, predicates_, functions_);
}

bool
ProblemExpert::clearGoal()
{
  goal_.nodes.clear();
  return true;
}

bool
ProblemExpert::clearKnowledge()
{
  instances_.clear();
  predicates_.clear();
  functions_.clear();
  return true;
}

bool
ProblemExpert::isValidType(const std::string & type)
{
  auto valid_types = domain_expert_->getTypes();
  auto it = std::find(valid_types.begin(), valid_types.end(), type);

  return it != valid_types.end();
}

bool
ProblemExpert::existInstance(const std::string & name)
{
  bool found = false;
  int i = 0;

  while (!found && i < instances_.size()) {
    if (instances_[i].name == name) {
      found = true;
    }
    i++;
  }

  return found;
}

bool
ProblemExpert::existPredicate(const plansys2::Predicate & predicate)
{
  bool found = false;
  int i = 0;

  while (!found && i < predicates_.size()) {
    if (parser::pddl::checkNodeEquality(predicates_[i], predicate)) {
      found = true;
    }
    i++;
  }

  return found;
}

bool
ProblemExpert::existFunction(const plansys2::Function & function)
{
  bool found = false;
  int i = 0;

  while (!found && i < functions_.size()) {
    if (parser::pddl::checkNodeEquality(functions_[i], function)) {
      found = true;
    }
    i++;
  }

  return found;
}

bool
ProblemExpert::isValidPredicate(const plansys2::Predicate & predicate)
{
  bool valid = false;

  const std::optional<plansys2::Predicate> & model_predicate =
    domain_expert_->getPredicate(predicate.name);
  if (model_predicate) {
    if (model_predicate.value().parameters.size() == predicate.parameters.size()) {
      bool same_types = true;
      int i = 0;
      while (same_types && i < predicate.parameters.size()) {
        auto arg_type = getInstance(predicate.parameters[i].name);

        if (!arg_type.has_value()) {
          same_types = false;
        } else if (arg_type.value().type != model_predicate.value().parameters[i].type) {
          bool isSubtype = false;
          for (std::string subType : model_predicate.value().parameters[i].sub_types) {
            if (arg_type.value().type == subType) {
              isSubtype = true;
              break;
            }
          }
          if (!isSubtype) {
            same_types = false;
          }
        }
        i++;
      }
      valid = same_types;
    }
  }

  return valid;
}

bool
ProblemExpert::isValidFunction(const plansys2::Function & function)
{
  bool valid = false;

  const std::optional<plansys2::Function> & model_function =
    domain_expert_->getFunction(function.name);
  if (model_function) {
    if (model_function.value().parameters.size() == function.parameters.size()) {
      bool same_types = true;
      int i = 0;
      while (same_types && i < function.parameters.size()) {
        auto arg_type = getInstance(function.parameters[i].name);

        if (!arg_type.has_value()) {
          same_types = false;
        } else if (arg_type.value().type != model_function.value().parameters[i].type) {
          bool isSubtype = false;
          for (std::string subType : model_function.value().parameters[i].sub_types) {
            if (arg_type.value().type == subType) {
              isSubtype = true;
              break;
            }
          }
          if (!isSubtype) {
            same_types = false;
          }
        }
        i++;
      }
      valid = same_types;
    }
  }

  return valid;
}

bool
ProblemExpert::isValidGoal(const plansys2::Goal & goal)
{
  return checkPredicateTreeTypes(goal, domain_expert_);
}

bool
ProblemExpert::checkPredicateTreeTypes(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<DomainExpert> & domain_expert,
  uint8_t node_id)
{
  if (node_id >= tree.nodes.size()) {
    return false;
  }

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::AND: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::OR: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::NOT: {
        return checkPredicateTreeTypes(tree, domain_expert, tree.nodes[node_id].children[0]);
      }

    case plansys2_msgs::msg::Node::PREDICATE: {
        return isValidPredicate(tree.nodes[node_id]);
      }

    case plansys2_msgs::msg::Node::FUNCTION: {
        return isValidFunction(tree.nodes[node_id]);
      }

    case plansys2_msgs::msg::Node::EXPRESSION: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::NUMBER: {
        return true;
      }

    default:
      // LCOV_EXCL_START
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion [" <<
        parser::pddl::toString(tree, node_id) << "]" << std::endl;
      // LCOV_EXCL_STOP
  }

  return false;
}

std::string
ProblemExpert::getProblem()
{
  parser::pddl::Domain domain(domain_expert_->getDomain());
  parser::pddl::Instance problem(domain);

  problem.name = "problem_1";

  for (const auto & instance : instances_) {
    problem.addObject(instance.name, instance.type);
  }

  for (plansys2_msgs::msg::Node predicate : predicates_) {
    StringVec v;

    for (size_t i = 0; i < predicate.parameters.size(); i++) {
      v.push_back(predicate.parameters[i].name);
    }

    std::transform(predicate.name.begin(), predicate.name.end(), predicate.name.begin(), ::tolower);

    problem.addInit(predicate.name, v);
  }

  for (plansys2_msgs::msg::Node function : functions_) {
    StringVec v;

    for (size_t i = 0; i < function.parameters.size(); i++) {
      v.push_back(function.parameters[i].name);
    }

    std::transform(
      function.name.begin(), function.name.end(),
      function.name.begin(), ::tolower);

    problem.addInit(function.name, function.value, v);
  }

  std::vector<plansys2_msgs::msg::Node> predicates;
  parser::pddl::getPredicates(predicates, goal_);

  for (auto predicate : predicates) {
    StringVec v;

    for (size_t i = 0; i < predicate.parameters.size(); i++) {
      v.push_back(predicate.parameters[i].name);
    }

    std::transform(predicate.name.begin(), predicate.name.end(), predicate.name.begin(), ::tolower);

    problem.addGoal(predicate.name, v);
  }

  std::ostringstream stream;
  stream << problem;
  return stream.str();
}

bool
ProblemExpert::addProblem(const std::string & problem_str)
{
  if (problem_str.empty()) {
    std::cerr << "Empty problem." << std::endl;
    return false;
  }
  parser::pddl::Domain domain(domain_expert_->getDomain());

  std::string lc_problem = problem_str;
  std::transform(
    problem_str.begin(), problem_str.end(), lc_problem.begin(),
    [](unsigned char c) {return std::tolower(c);});

  lc_problem = remove_comments(lc_problem);

  std::cout << "Domain:\n" << domain << std::endl;
  std::cout << "Problem:\n" << lc_problem << std::endl;

  parser::pddl::Instance problem(domain);

  std::string domain_name = problem.getDomainName(lc_problem);
  if (domain_name.empty()) {
    std::cerr << "Domain name is empty" << std::endl;
    return false;
  } else if (!domain_expert_->existDomain(domain_name)) {
    std::cerr << "Domain name does not exist: " << domain_name << std::endl;
    return false;
  }

  domain.name = domain_name;
  try {
    problem.parse(lc_problem);
  } catch (std::runtime_error ex) {
    // all errors thrown by the Stringreader object extend std::runtime_error
    std::cerr << ex.what() << std::endl;
    return false;
  }

  std::cout << "Parsed problem: " << problem << std::endl;

  for (unsigned i = 0; i < domain.types.size(); ++i) {
    if (domain.types[i]->objects.size() ) {
      for (unsigned j = 0; j < domain.types[i]->objects.size(); ++j) {
        plansys2::Instance instance;
        instance.name = domain.types[i]->objects[j];
        instance.type = domain.types[i]->name;
        std::cout << "Adding instance: " << instance.name << " " << instance.type << std::endl;
        addInstance(instance);
      }
    }
  }

  plansys2_msgs::msg::Tree tree;
  for (auto ground : problem.init) {
    auto tree_node = ground->getTree(tree, domain);
    switch (tree_node->node_type) {
      case plansys2_msgs::msg::Node::PREDICATE: {
          plansys2::Predicate pred_node(*tree_node);
          std::cout << "Adding predicate: " <<
            parser::pddl::toString(tree, tree_node->node_id) << std::endl;
          if (!addPredicate(pred_node)) {
            std::cerr << "Failed to add predicate: " << parser::pddl::toString(
              tree,
              tree_node->node_id) <<
              std::endl;
          }
        }
        break;
      case plansys2_msgs::msg::Node::FUNCTION: {
          plansys2::Function func_node(*tree_node);
          std::cout << "Adding function: " <<
            parser::pddl::toString(tree, tree_node->node_id) << std::endl;
          if (!addFunction(func_node)) {
            std::cerr << "Failed to add function: " << parser::pddl::toString(
              tree,
              tree_node->node_id) <<
              std::endl;
          }
        }
        break;
      default:
        break;
    }
  }

  plansys2_msgs::msg::Node node;
  node.node_type = plansys2_msgs::msg::Node::AND;
  node.node_id = 0;
  node.negate = false;

  plansys2_msgs::msg::Tree goal;
  goal.nodes.push_back(node);
  for (auto ground : problem.goal) {
    auto goal_node = ground->getTree(goal, domain);
    std::cout << "Adding goal node: " <<
      parser::pddl::toString(goal, goal_node->node_id) << std::endl;
    goal.nodes[0].children.push_back(goal_node->node_id);
  }
  std::cout << "Adding Goal: " << parser::pddl::toString(goal) << std::endl;
  setGoal(goal);

  return true;
}

}  // namespace plansys2
