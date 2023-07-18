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

#ifndef SET_POSES_NODE_HPP
#define SET_POSES_NODE_HPP

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"

#include "any_robot_pick_n_place/load_parameters.hpp"
#include <behaviortree_cpp/bt_factory.h>

class SetPosesNode : public BT::SyncActionNode
{
  protected:
    rclcpp::Node::SharedPtr node_;
    moveit_cpp::MoveItCppPtr moveit_cpp_;
    moveit_cpp::PlanningComponentPtr planning_component_;
    // Parameters params_; Load the parameters at manager node and pass it to this node
    geometry_msgs::msg::Pose pick_pose_;
    geometry_msgs::msg::Pose place_pose_;
    std::string planning_goup_name_;
    std::string planning_base_frame_;
    std::string planning_end_effector_frame_;

  public:
    SetPosesNode(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        moveit_cpp_ = config.blackboard->get<moveit_cpp::MoveItCppPtr>("moveit_cpp");
        planning_component_ = config.blackboard->get<moveit_cpp::PlanningComponentPtr>("planning_component");
        pick_pose_ = config.blackboard->get<geometry_msgs::msg::Pose>("pick_pose");
        place_pose_ = config.blackboard->get<geometry_msgs::msg::Pose>("place_pose");
        planning_goup_name_ = config.blackboard->get<std::string>("planning_group_name");
        planning_base_frame_ = config.blackboard->get<std::string>("planning_base_frame");
        planning_end_effector_frame_ = config.blackboard->get<std::string>("planning_end_effector_frame");
    }

    static BT::PortsList providedPorts()
    {
        const char *op_description = "Pick or Place";
        return {
            BT::InputPort<std::string>("description"),
            BT::InputPort<std::string>("operation", op_description),
            BT::InputPort<double>("offset_z", "Offset in z direction"),
            BT::OutputPort<moveit::core::RobotStatePtr>("robot_state"),
        };
    }

    BT::NodeStatus tick() override
    {
        if (!getInput<std::string>("description").has_value())
        {
            RCLCPP_INFO(node_->get_logger(), "%s", getInput<std::string>("description").value().c_str());
        }

        if (!getInput<std::string>("operation").has_value())
        {
            RCLCPP_ERROR(node_->get_logger(), "operation not set. Please set the operation to Pick or Place");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "Setting poses for %s", getInput<std::string>("operation").value().c_str());
        geometry_msgs::msg::Pose pose;
        if (getInput<std::string>("operation").value() == "Pick")
        {
            pose = pick_pose_;
        }
        else if (getInput<std::string>("operation").value() == "Place")
        {
            pose = place_pose_;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Operation not set. Please set the operation to Pick or Place");
            return BT::NodeStatus::FAILURE;
        }

        if (getInput<double>("offset_z").has_value())
        {
            pose.position.z -= getInput<double>("offset_z").value();
        }

        auto state = moveit_cpp_->getCurrentState();
        auto jmg = state->getJointModelGroup(planning_goup_name_);
        bool found_ik = false;

        auto transform = state->getFrameTransform(planning_base_frame_, &found_ik);
        if (!found_ik)
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not find transform from %s to %s", planning_base_frame_.c_str(),
                         jmg->getLinkModelNames().back().c_str());
            return BT::NodeStatus::FAILURE;
        }

        Eigen::Isometry3d target_pose;
        tf2::fromMsg(pose, target_pose);
        // From planning frame to model frame
        target_pose = transform * target_pose;
        pose = tf2::toMsg(target_pose);
        RCLCPP_INFO(node_->get_logger(), "Pose in planning frame: %s", geometry_msgs::msg::to_yaml(pose).c_str());

        if (!state->setFromIK(jmg, target_pose, planning_end_effector_frame_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not find IK solution");
            return BT::NodeStatus::FAILURE;
        }

        // Collition check
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
            RCLCPP_ERROR(node_->get_logger(), "Collision detected");
            return BT::NodeStatus::FAILURE;
        }

        setOutput("robot_state", state);
        return BT::NodeStatus::SUCCESS;
    }
};

#endif // SET_POSES_NODE_HPP
