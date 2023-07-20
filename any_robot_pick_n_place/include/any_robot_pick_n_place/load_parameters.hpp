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

#ifndef LOAD_PARAMETERS_HPP
#define LOAD_PARAMETERS_HPP

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include <rosparam_shortcuts/rosparam_shortcuts.h>

struct Parameters
{
    std::string arm_group_name;
    std::string planning_frame;
    std::string moveing_link;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;

    void load(const rclcpp::Node::SharedPtr &node)
    {
        RCLCPP_INFO(node->get_logger(), "Loading parameters");
        node->get_parameter("arm_group_name", arm_group_name);
        node->get_parameter("planning_frame", planning_frame);
        node->get_parameter("moveing_link", moveing_link);

        RCLCPP_INFO(node->get_logger(), "Loaded parameters arm_group_name: %s, planning_frame: %s, moveing_link: %s",
                    arm_group_name.c_str(), planning_frame.c_str(), moveing_link.c_str());

        size_t errors = 0;
        errors += !rosparam_shortcuts::get(node, "pick_pose", pick_pose);
        errors += !rosparam_shortcuts::get(node, "place_pose", place_pose);
        rosparam_shortcuts::shutdownIfError(errors);
    }
};

#endif
