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
    }

    void getPoseCallback(const color_pose_msgs::msg::ColorPose& msg)
    {
      RCLCPP_INFO(node_->get_logger(), "Checking for color pose suggestion");
      color_pose_ = msg;
      is_new_pose = true;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("color", "Red", "Red | Yellow | Green | Blue"),
            BT::InputPort<double>("std_pick_z", 0.02, "Standard Pick z position"),
            BT::InputPort<double>("pre_z_offset", 0.12, "Offset for pre in z-axis."),
            BT::OutputPort<moveit::core::RobotStatePtr>("robot_state"),
        };
    }

    BT::NodeStatus onStart() override
    {
        RCLCPP_INFO(node_->get_logger(), "Initialising Color Detection.");
        if(!getInput<std::string>("color").has_value())
        {
            RCLCPP_ERROR(node_->get_logger(), "Choose your color choise: %s", getInput<std::string>("color").error().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if(!getInput<std::string>("std_pick_z").has_value())
        {
            RCLCPP_ERROR(node_->get_logger(), "Standard pick z axis position: %s", getInput<double>("std_pick_z").error().c_str());
            return BT::NodeStatus::FAILURE;
        }
        selected_color_ = getInput<std::string>("color").value();
        sub_color_pose_ = node_->create_subscription<color_pose_msgs::msg::ColorPose>(
            "color_pose_estimation/color_pose", 30, std::bind(&DetectColorNode::getPoseCallback, this, std::placeholders::_1));
        is_new_pose = false;
        rclcpp::sleep_for(std::chrono::seconds(1));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        RCLCPP_INFO(node_->get_logger(), "Initialising Color Detection.");

        if(!is_new_pose)
        {
            return BT::NodeStatus::RUNNING;
        }
        if(color_pose_.color != selected_color_)
        {
            is_new_pose = false;
            return BT::NodeStatus::RUNNING;
        }
        is_new_pose = false;
        RCLCPP_INFO(node_->get_logger(), "Received right color pose.");

        geometry_msgs::msg::PoseStamped pick_pose;
        tf2::Quaternion q;

        pick_pose.pose.position = color_pose_.pose.position;
        pick_pose.pose.orientation = home_pose_.orientation;

        pick_pose.pose.position.z = getInput<double>("std_pick_z").value();
        
        if(getInput<double>("pre_z_offset").has_value())
        {
            pick_pose.pose.position.z += getInput<double>("pre_z_offset").value();
        }

        auto robot_state = moveit_cpp_->getCurrentState();
        auto jmg1 = robot_state->getJointModelGroup(planning_goup_name_);

        if (!robot_state->setFromIK(jmg1, pick_pose.pose, 10))
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not find IK solution for pick pose.");
            return BT::NodeStatus::FAILURE;
        }
    
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.contacts = false;
        collision_request.cost = false;
        collision_request.distance = false;

        {
            auto planning_scene_monitor =
                planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp_->getPlanningSceneMonitor());
            planning_scene_monitor->checkCollision(collision_request, collision_result, *robot_state);
        }

        if (collision_result.collision)
        {
            RCLCPP_ERROR(node_->get_logger(), "Collision detected for pick pose.");
            return BT::NodeStatus::FAILURE;
        }

        setOutput("pick_robot_state", robot_state);
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        RCLCPP_INFO(node_->get_logger(), "Halting color detection.");
        error_.store(true);
        timer_->cancel();
    }
};

#endif // DETECT_COLOR_NODE_HPP
