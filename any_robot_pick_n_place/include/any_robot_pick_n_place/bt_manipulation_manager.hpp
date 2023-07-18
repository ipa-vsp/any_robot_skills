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

#ifndef BT_MANIPULATION_MANAGER_HPP
#define BT_MANIPULATION_MANAGER_HPP

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"
#include "rclcpp/rclcpp.hpp"

#include "any_robot_pick_n_place/execute_motion_node.hpp"
#include "any_robot_pick_n_place/load_parameters.hpp"
#include "any_robot_pick_n_place/plan_motion_node.hpp"
#include "any_robot_pick_n_place/set_poses_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviortree_cpp/bt_factory.h"

class BTManipulationManager
{
  protected:
    rclcpp::Node::SharedPtr node_;
    moveit_cpp::MoveItCppPtr moveit_cpp_;
    moveit_cpp::PlanningComponentPtr planning_component_;

    std::shared_ptr<BT::BehaviorTreeFactory> factory_;
    std::shared_ptr<BT::NodeConfiguration> config_;
    std::shared_ptr<BT::Tree> tree_;

    Parameters param_;

  public:
    BTManipulationManager(rclcpp::Node::SharedPtr node, const Parameters &param) : node_(node), param_(param)
    {
        // param_.load(node_);
        moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
        moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();
        planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(param_.arm_group_name, moveit_cpp_);
        factory_ = std::make_shared<BT::BehaviorTreeFactory>();
        node_->declare_parameter("behavior_tree_file",
                                 ament_index_cpp::get_package_share_directory("any_robot_pick_n_place") +
                                     "/config/simple_pick_n_place.xml");
    }

    void start_bt()
    {
        config_ = std::make_shared<BT::NodeConfiguration>();
        config_->blackboard = BT::Blackboard::create();
        config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
        config_->blackboard->set<moveit_cpp::MoveItCppPtr>("moveit_cpp", moveit_cpp_);
        config_->blackboard->set<moveit_cpp::PlanningComponentPtr>("planning_component", planning_component_);
        config_->blackboard->set<std::string>("planning_group_name", param_.arm_group_name);
        config_->blackboard->set<std::string>("planning_base_frame", param_.planning_frame);
        config_->blackboard->set<std::string>("planning_end_effector_frame", param_.moveing_link);

        config_->blackboard->set<geometry_msgs::msg::Pose>("pick_pose", param_.pick_pose);
        config_->blackboard->set<geometry_msgs::msg::Pose>("place_pose", param_.place_pose);

        BT::NodeBuilder set_pose_builder = [](const std::string &name, const BT::NodeConfiguration &config)
        { return std::make_unique<SetPosesNode>(name, config); };
        BT::NodeBuilder plan_motion_builder = [](const std::string &name, const BT::NodeConfiguration &config)
        { return std::make_unique<PlanMotionNode>(name, config); };
        BT::NodeBuilder execute_motion_builder = [](const std::string &name, const BT::NodeConfiguration &config)
        { return std::make_unique<ExecuteMotionNode>(name, config); };

        try
        {
            factory_->registerBuilder<SetPosesNode>("SetPoses", set_pose_builder);
            factory_->registerBuilder<PlanMotionNode>("PlanMotion", plan_motion_builder);
            factory_->registerBuilder<ExecuteMotionNode>("ExecuteMotion", execute_motion_builder);
        }
        catch (BT::BehaviorTreeException &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Behavior Tree Exception: %s", e.what());
            throw e;
        }

        std::string behavior_tree_file = node_->get_parameter("behavior_tree_file").as_string();
        tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromFile(behavior_tree_file, config_->blackboard));

        auto status = tree_->tickOnce();

        while (rclcpp::ok() && (status == BT::NodeStatus::RUNNING || status != BT::NodeStatus::FAILURE))
        {
            status = tree_->tickOnce();
            // rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

#endif // BT_MANIPULATION_MANAGER_HPP
