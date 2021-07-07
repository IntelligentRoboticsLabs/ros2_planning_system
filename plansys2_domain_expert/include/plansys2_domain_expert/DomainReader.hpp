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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINREADER_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINREADER_HPP_

#include <string>
#include <vector>

namespace plansys2
{

struct Domain
{
  std::string name;
  std::string requirements;
  std::string types;
  std::string constants;
  std::string predicates;
  std::string functions;
  std::vector<std::string> actions;
};

class DomainReader
{
public:
  DomainReader();

  void add_domain(const std::string & domain);
  std::string get_joint_domain() const;
  std::vector<Domain> get_domains() {return domains_;}

protected:
  int get_end_block(const std::string & domain, std::size_t init_pos);

  std::string get_name(std::string & domain);
  std::string get_requirements(std::string & domain);
  std::string get_types(const std::string & domain);
  std::string get_constants(const std::string & domain);
  std::string get_predicates(const std::string & domain);
  std::string get_functions(const std::string & domain);
  std::vector<std::string> get_actions(const std::string & domain);

private:
  std::vector<Domain> domains_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINREADER_HPP_
