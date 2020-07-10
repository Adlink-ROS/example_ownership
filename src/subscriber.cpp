// Copyright 2019 ADLINK Technology, Inc. Advanced Robotic Platform Group
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
#include <memory>

class OwnershipSubscriberNode : public rclcpp::Node {
  public:
    OwnershipSubscriberNode(enum cyclonedds_options::qos_ownership_kind kind)
      : Node("Ownership_sub")
    {
      rclcpp::SystemDefaultsQoS qos;

      auto callback =
        [this](const nsdk_ownership::msg::Ownership::SharedPtr msg) -> void {
          RCLCPP_INFO(this->get_logger(), "I heard: id: %d, strength: %d, value: %d",
              msg->id, msg->strength, msg->value);
        };

      std::unique_ptr<cyclonedds_options::SubOptions> cyclonedds_options(new cyclonedds_options::SubOptions());
      cyclonedds_options->setOwnership(kind);
      rclcpp::SubscriptionOptions so;
      so.rmw_implementation_payload = std::move(cyclonedds_options);

      sub_ = this->create_subscription<nsdk_ownership::msg::Ownership>("ownership",
          qos, callback, so);
    }

  private:
    rclcpp::Subscription<nsdk_ownership::msg::Ownership>::SharedPtr sub_;
};

void print_usage()
{
  printf("Usage for Neuron SDK ownership subscriber:\n");
  printf("ownership_sub [-k ownership_kind] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-k ownership_role : Specify ownership role, should be <shared> or <exclusive>. Defaults to shared.\n");
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

  auto node = std::make_shared<OwnershipSubscriberNode>(kind);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}