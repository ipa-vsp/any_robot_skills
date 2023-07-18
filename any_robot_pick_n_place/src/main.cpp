// Copyright 2023 Vishnuprasad Prachandabhanu
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

#include "any_robot_pick_n_place/bt_manipulation_manager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("any_robot_pick_n_place_node", options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_shared<std::thread>(
        [&executor, &node]()
        {
            executor.add_node(node->get_node_base_interface());
            executor.spin();
            executor.remove_node(node->get_node_base_interface());
        });
    rclcpp::sleep_for(std::chrono::seconds(1));
    BTManipulationManager manager(node);
    manager.start_bt();
    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
    spin_thread->join();
    return 0;
}
