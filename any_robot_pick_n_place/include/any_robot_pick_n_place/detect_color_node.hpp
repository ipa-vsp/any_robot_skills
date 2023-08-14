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

#ifndef DETECT_COLOR_NODE_HPP
#define DETECT_COLOR_NODE_HPP

#include <chrono>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <fstream>
#include <iostream>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <color_pose_msgs/msg/color_pose.hpp>

#include <behaviortree_cpp/bt_factory.h>

class DetectColorNode : public BT::StatefulActionNode
{
  protected:
    std::atomic_int required_poses_ = 3;
    rclcpp::Node::SharedPtr node_;
    // color_pose_msgs::msg::ColorPose final_pose_;
    std::atomic_int counter_;
    std::atomic_bool error_;
    rclcpp::TimerBase::SharedPtr timer_;
    moveit_cpp::MoveItCppPtr moveit_cpp_;
    moveit_cpp::PlanningComponentPtr planning_component_;
    std::string planning_goup_name_;
    std::string planning_base_frame_;
    std::string planning_end_effector_frame_;

    geometry_msgs::msg::Pose home_pose_;

    color_pose_msgs::msg::ColorPose color_pose_;
    rclcpp::Subscription<color_pose_msgs::msg::ColorPose>::SharedPtr sub_color_pose_;
    std::atomic_bool is_new_pose;
    std::string selected_color_;

  public:
    DetectColorNode(const std::string &name, const BT::NodeConfiguration &config) : BT::StatefulActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        RCLCPP_INFO(node_->get_logger(), "Initialising Execute Node.");
        sleep(1.0);

        moveit_cpp_ = config.blackboard->get<moveit_cpp::MoveItCppPtr>("moveit_cpp");
        planning_component_ = config.blackboard->get<moveit_cpp::PlanningComponentPtr>("planning_component");
        planning_goup_name_ = config.blackboard->get<std::string>("planning_group_name");
        planning_base_frame_ = config.blackboard->get<std::string>("planning_base_frame");
        planning_end_effector_frame_ = config.blackboard->get<std::string>("planning_end_effector_frame");

        home_pose_ = config.blackboard->get<geometry_msgs::msg::Pose>("home_pose");
        sub_color_pose_ = node_->create_subscription<color_pose_msgs::msg::ColorPose>(
            "color_pose_estimation/color_pose", 30,
            std::bind(&DetectColorNode::getPoseCallback, this, std::placeholders::_1));
    }

    void getPoseCallback(const color_pose_msgs::msg::ColorPose &msg)
    {
        // RCLCPP_INFO(node_->get_logger(), "Checking for color pose suggestion");
        color_pose_ = msg;
        is_new_pose = true;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("color", "Red", "Red | Yellow | Green | Blue"),
            BT::InputPort<double>("std_pick_z", 0.02, "Standard Pick z position"),
            BT::InputPort<double>("pre_z_offset", 0.12, "Offset for pre in z-axis."),
            BT::OutputPort<moveit::core::RobotStatePtr>("pre_pick_robot_state"),
            BT::OutputPort<moveit::core::RobotStatePtr>("pick_robot_state"),
        };
    }

    BT::NodeStatus onStart() override
    {
        RCLCPP_INFO(node_->get_logger(), "Initialising Color Detection.");
        if (!getInput<std::string>("color").has_value())
        {
            RCLCPP_ERROR(node_->get_logger(), "Choose your color choice: %s",
                         getInput<std::string>("color").error().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput<double>("std_pick_z").has_value() || !getInput<double>("pre_z_offset"))
        {
            RCLCPP_ERROR(node_->get_logger(), "Standard pick z axis and offset positions are not set: %s",
                         getInput<double>("std_pick_z").error().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if(!getInput<std::string>("color").has_value())
        {
            RCLCPP_ERROR(node_->get_logger(), "Color is not selected: %s", getInput<std::string>("color").error().c_str());
            return BT::NodeStatus::FAILURE;
        }
        selected_color_ = getInput<std::string>("color").value();
        is_new_pose = false;
        rclcpp::sleep_for(std::chrono::seconds(1));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {

        if (!is_new_pose)
        {
            return BT::NodeStatus::RUNNING;
        }
        if (color_pose_.color != selected_color_)
        {
            is_new_pose = false;
            return BT::NodeStatus::RUNNING;
        }
        is_new_pose = false;
        RCLCPP_INFO(node_->get_logger(), "Received right color pose: %s", selected_color_.c_str());

        geometry_msgs::msg::Pose pick_pose, pre_pick_pose;
        tf2::Quaternion q;

        pick_pose.position.x = -0.144;
        pick_pose.position.y = -0.080; // color_pose_.pose.position;
        pick_pose.position.z = getInput<double>("std_pick_z").value();
        pick_pose.orientation = home_pose_.orientation; // Todo: Currently fixed orientation remove this in the future

        pre_pick_pose = pick_pose;
        pre_pick_pose.position.z += getInput<double>("pre_z_offset").value();

        moveit::core::RobotStatePtr pre_pick_robot_state, pick_robot_state;

        if(proccess_poses(pre_pick_pose, "Pre Or Post Pick", pre_pick_robot_state) == BT::NodeStatus::FAILURE)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to process poses for pre or post pick");
            return BT::NodeStatus::FAILURE;
        }
        if(proccess_poses(pick_pose, "Pick", pick_robot_state) == BT::NodeStatus::FAILURE)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to process poses for pick");
            return BT::NodeStatus::FAILURE;
        }

        setOutput("pick_robot_state", pre_pick_robot_state);
        setOutput("pick_robot_state", pick_robot_state);
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        RCLCPP_INFO(node_->get_logger(), "Halting color detection.");
        error_.store(true);
        timer_->cancel();
    }

    BT::NodeStatus proccess_poses(geometry_msgs::msg::Pose pose, const std::string move_action, moveit::core::RobotStatePtr &robot_state)
    {
        auto state = moveit_cpp_->getCurrentState();
        auto jmg = state->getJointModelGroup(planning_goup_name_);

        bool found_ik = false;

        auto transform = state->getFrameTransform(planning_base_frame_, &found_ik);

        if (!found_ik)
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not find transform from %s to %s for %s", planning_base_frame_.c_str(),
                         jmg->getLinkModelNames().back().c_str(), move_action.c_str());
            return BT::NodeStatus::FAILURE;
        }

        Eigen::Isometry3d target_pose;
        tf2::fromMsg(pose, target_pose);
        // From planning frame to model frame
        target_pose = transform * target_pose;
        pose = tf2::toMsg(target_pose);
        RCLCPP_INFO(node_->get_logger(), "%s Pose in planning frame: %s", move_action.c_str(), geometry_msgs::msg::to_yaml(pose).c_str());

        if (!state->setFromIK(jmg, target_pose, planning_end_effector_frame_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not find IK solution for %s pose", move_action.c_str());
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_ERROR(node_->get_logger(), "Checking collision ....!");
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.contacts = false;
        collision_request.cost = false;
        collision_request.distance = false;

        {
            auto planning_scene_monitor =
                planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp_->getPlanningSceneMonitor());
            planning_scene_monitor->checkCollision(collision_request, collision_result, *state);
        }

        if (collision_result.collision)
        {
            RCLCPP_ERROR(node_->get_logger(), "Collision detected for %s pose.", move_action.c_str());
            return BT::NodeStatus::FAILURE;
        }
        robot_state = state;
        return BT::NodeStatus::SUCCESS;
    }    
};

#endif // DETECT_COLOR_NODE_HPP
