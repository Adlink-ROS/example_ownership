// Copyright 2019 ADLINK Technology, Inc.
// Developer: Alan Chen (alan.chen@adlinktech.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rcutils/snprintf.h"
#include "nsdk_ownership/msg/ownership.hpp"
#include "cyclonedds_options/cyclonedds_options.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class OwnershipPublisherNode : public rclcpp::Node {
  public:
    OwnershipPublisherNode(enum cyclonedds_options::qos_ownership_kind kind, uint32_t strength, uint32_t id_in)
      : Node("Ownership_pub"),
        id_(id_in),
        strength_(strength)
    {
      rclcpp::SystemDefaultsQoS qos;

      msg_ = std::make_shared<nsdk_ownership::msg::Ownership>();

      std::unique_ptr<cyclonedds_options::PubOptions> cyclonedds_options(new cyclonedds_options::PubOptions());
      cyclonedds_options->setOwnership(kind, strength);
      rclcpp::PublisherOptions po;
      po.rmw_implementation_payload = std::move(cyclonedds_options);
      
      pub_ = this->create_publisher<nsdk_ownership::msg::Ownership>(
          "ownership", qos, po);

      timer_ = this->create_wall_timer(
          1s,
          [this](void) -> void {
          msg_->id = id_;
          msg_->strength = strength_;
          msg_->value = counter_;
          pub_->publish(*msg_);
          counter_ ++;
          RCLCPP_INFO(this->get_logger(), "Publishing msg. id: %d, strength: %d, value: %d",
              msg_->id, msg_->strength, msg_->value);
          });
    }

  private:
    uint8_t id_ = 0;
    uint16_t strength_ = 0;
    uint32_t counter_ = 0;
    rclcpp::Publisher<nsdk_ownership::msg::Ownership>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<nsdk_ownership::msg::Ownership> msg_;
};

void print_usage()
{
  printf("Usage for Neuron SDK ownership publisher:\n");
  printf("ownership_pub [-k ownership_kind] [-s strength] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-k ownership_role : Specify ownership role, should be <shared> or <exclusive>. Defaults to shared.\n");
  printf("-s strength: ownership strength\n");
  printf("-i id: node id\n");
}

int main(int argc, char* argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  rclcpp::init(argc, argv);

  enum cyclonedds_options::qos_ownership_kind kind;
  if (rcutils_cli_option_exist(argv, argv + argc, "-k")) {
    char* ownership_role = rcutils_cli_get_option(argv, argv + argc, "-k");
    if (ownership_role != nullptr) {
      std::string ownership_role_ = std::string(ownership_role);
      if (ownership_role_ == "shared") {
        kind = cyclonedds_options::QOS_POLICY_OWNERSHIP_SHARED;
      } else if (ownership_role_ == "exclusive") {
        kind = cyclonedds_options::QOS_POLICY_OWNERSHIP_EXCLUSIVE;
      } else {
        printf("Unknown ownership role\n");
        print_usage();
        return 0;
      }
    }
  }

  uint32_t id = 0;
  if (rcutils_cli_option_exist(argv, argv + argc, "-i")) {
    char *id_chr = rcutils_cli_get_option(argv, argv + argc, "-i");
    if (id_chr != nullptr) {
      id = std::atoi(id_chr);
    }
  }

  uint32_t strength = 0;
  if (rcutils_cli_option_exist(argv, argv + argc, "-s")) {
    char *strength_chr = rcutils_cli_get_option(argv, argv + argc, "-s");
    if (strength_chr != nullptr) {
      strength = std::atoi(strength_chr);
    }
  }

  auto node = std::make_shared<OwnershipPublisherNode>(kind, strength, id);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}