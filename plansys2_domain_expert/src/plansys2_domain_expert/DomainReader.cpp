// Copyright 2020 Intelligent Robotics Lab
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

#include "plansys2_domain_expert/DomainReader.hpp"

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <set>

#include "plansys2_core/Utils.hpp"


namespace plansys2
{

DomainReader::DomainReader()
{
}

void
DomainReader::add_domain(const std::string & domain)
{
  if (domain.empty()) {
    std::cerr << "Empty domain" << std::endl;
    return;
  }

  Domain new_domain;

  std::string lc_domain = domain;
  std::transform(
    domain.begin(), domain.end(), lc_domain.begin(),
    [](unsigned char c) {return std::tolower(c);});

  lc_domain = remove_comments(lc_domain);

  new_domain.name = get_name(lc_domain);
  new_domain.requirements = get_requirements(lc_domain);
  new_domain.types = get_types(lc_domain);
  new_domain.constants = get_constants(lc_domain);
  new_domain.predicates = get_predicates(lc_domain);
  new_domain.functions = get_functions(lc_domain);
  new_domain.actions = get_actions(lc_domain);

  domains_.push_back(new_domain);
}

std::string
DomainReader::get_joint_domain() const
{
  std::string ret = "(define (domain plansys2)\n";

  ret += "(:requirements ";

  std::set<std::string> reqs_set;
  for (auto & domain : domains_) {
    std::vector<std::string> reqs = tokenize(domain.requirements, " ");
    reqs_set.insert(reqs.begin(), reqs.end());
  }
  for (auto & req : reqs_set) {
    ret += req + " ";
  }
  ret += ")\n\n";

  ret += "(:types\n";
  for (auto & domain : domains_) {
    if (!domain.types.empty()) {
      ret += domain.types + "\n";
    }
  }
  ret += ")\n\n";

  ret += "(:constants\n";
  for (auto & domain : domains_) {
    if (!domain.constants.empty()) {
      ret += domain.constants + "\n";
    }
  }
  ret += ")\n\n";

  ret += "(:predicates\n";
  std::set<std::string> preds_set;
  for (auto & domain : domains_) {
    std::vector<std::string> preds = tokenize(domain.predicates, "\n");
    preds_set.insert(preds.begin(), preds.end());
  }
  for (auto & pred : preds_set) {
    ret += pred + "\n";
  }
  ret += ")\n\n";

  ret += "(:functions\n";
  for (auto & domain : domains_) {
    if (!domain.functions.empty()) {
      ret += domain.functions + "\n";
    }
  }
  ret += ")\n\n";

  for (auto & domain : domains_) {
    for (auto & action : domain.actions) {
      if (!action.empty()) {
        ret += action + "\n";
      }
    }
  }

  ret += ")\n";

  return ret;
}

int
DomainReader::get_end_block(const std::string & domain, std::size_t init_pos)
{
  std::size_t domain_length = domain.length();

  auto end_pos = init_pos;
  int p_counter = 1;
  while (end_pos < domain_length && p_counter > 0) {
    if (domain[end_pos] == '(') {
      p_counter++;
    } else if (domain[end_pos] == ')') {
      p_counter--;
    }

    if (p_counter > 0) {
      end_pos++;
    }
  }

  if (p_counter == 0) {
    return end_pos;
  } else {
    return -1;
  }
}

std::string
DomainReader::get_name(std::string & domain)
{
  const std::string pattern("domain");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    auto ret = substr_without_empty_lines(domain, init_pos, end_pos);

    // remove spaces
    ret.erase(std::remove(ret.begin(), ret.end(), ' '), ret.end());

    return ret;
  } else {
    return "";
  }
}

std::string
DomainReader::get_requirements(std::string & domain)
{
  const std::string pattern(":requirements");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    auto ret = substr_without_empty_lines(domain, init_pos, end_pos);
    // We remove the requirements part for not interfering with next analysis
    domain.replace(init_pos, end_pos - init_pos, "");

    return ret;
  } else {
    return "";
  }
}

std::string
DomainReader::get_types(const std::string & domain)
{
  const std::string pattern(":types");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    std::string ret = substr_without_empty_lines(domain, init_pos, end_pos);
    return ret;
  } else {
    return "";
  }
}


std::string
DomainReader::get_constants(const std::string & domain)
{
  const std::string pattern(":constants");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    std::string ret = substr_without_empty_lines(domain, init_pos, end_pos);
    return ret;
  } else {
    return "";
  }
}


std::string
DomainReader::get_predicates(const std::string & domain)
{
  const std::string pattern(":predicates");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    std::string ret = substr_without_empty_lines(domain, init_pos, end_pos);
    return ret;
  } else {
    return "";
  }
}

std::string
DomainReader::get_functions(const std::string & domain)
{
  const std::string pattern(":functions");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    std::string ret = substr_without_empty_lines(domain, init_pos, end_pos);
    return ret;
  } else {
    return "";
  }
}

std::vector<std::string>
DomainReader::get_actions(const std::string & domain)
{
  std::vector<std::string> ret;

  const std::string da_pattern(":durative-action");
  const std::string a_pattern(":action");

  auto ldomain = domain;

  size_t pos = std::string::npos;
  do {
    std::size_t da_pos = ldomain.find(da_pattern);
    std::size_t a_pos = ldomain.find(a_pattern);

    pos = std::min(da_pos, a_pos);

    if (pos != std::string::npos) {
      auto end_pos = get_end_block(ldomain, pos + 1);

      if (end_pos == -1) {
        break;
      }

      std::string lines = substr_without_empty_lines(ldomain, pos, end_pos + 1);
      ret.push_back("(" + lines);
      ldomain = ldomain.substr(end_pos + 1);
    }
  } while (!ldomain.empty() && pos != std::string::npos);

  return ret;
}


}  // namespace plansys2
