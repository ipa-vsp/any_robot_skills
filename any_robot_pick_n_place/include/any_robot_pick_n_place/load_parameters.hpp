#ifndef LOAD_PARAMETERS_HPP
#define LOAD_PARAMETERS_HPP

#include <string>
#include <vector>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <rosparam_shortcuts/rosparam_shortcuts.h>

struct Parameters
{
    std::string arm_group_name;
    geometry_msgs::msg::Pose object_pose;
    geometry_msgs::msg::Pose place_pose;

    void load(const rclcpp::Node::SharePtr & node)
    {
        node->get_parameter("arm_group_name", arm_group_name);
        
        size_t errors = 0;
        errors += !rosparam_shortcuts::get(node, "object_pose", object_pose);
        errors += !rosparam_shortcuts::get(node, "place_pose", place_pose);
        rosparam_shortcuts::shutdownIfError(errors);
    }

};

#endif