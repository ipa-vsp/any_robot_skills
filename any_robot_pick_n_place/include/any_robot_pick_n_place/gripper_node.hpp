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

#ifndef GRIPPER_NODE_HPP
#define GRIPPER_NODE_HPP

#include "control_msgs/action/gripper_command.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <behaviortree_cpp/bt_factory.h>

class CommandGripper : public BT::StatefulActionNode
{
  public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

    CommandGripper(const std::string &name, const BT::NodeConfiguration &config) : BT::StatefulActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        gripper_action_ = config.blackboard->get<std::string>("gripper_action");

        client_ = rclcpp_action::create_client<GripperCommand>(node_, gripper_action_);

        goal_options_.goal_response_callback =
            std::bind(&CommandGripper::goal_response_callback, this, std::placeholders::_1);
        goal_options_.feedback_callback =
            std::bind(&CommandGripper::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        goal_options_.result_callback = std::bind(&CommandGripper::result_callback, this, std::placeholders::_1);
    }

    static BT::PortsList providedPorts()
    {
        const char *op_description = "Gripper Command";
        return {BT::InputPort<std::string>("description", op_description),
                BT::InputPort<bool>("is_open", "true: open, false: close")};
    }

    BT::NodeStatus onStart() override
    {
        if (getInput<std::string>("description").has_value())
        {
            RCLCPP_INFO(node_->get_logger(), "Gripper: %s", getInput<std::string>("description").value().c_str());
        }

        if (!getInput<bool>("is_open").has_value())
        {
            RCLCPP_ERROR(node_->get_logger(), "Gripper command is not set. Exception thrown: %s",
                         getInput<bool>("is_open").error().c_str());
            return BT::NodeStatus::FAILURE;
        }

        bool is_open = getInput<bool>("is_open").value();

        if (!client_->wait_for_action_server(std::chrono::seconds(5)) && rclcpp::ok())
        {
            RCLCPP_ERROR(node_->get_logger(), "Can not connect to the action server: %s", gripper_action_.c_str());
            return BT::NodeStatus::FAILURE;
        }

        auto target = control_msgs::action::GripperCommand::Goal();
        if (!is_open)
        {
            target.command.position = 0.0;
        }
        else
        {
            target.command.position = 1.0;
        }

        exec_future = this->client_->async_send_goal(target, goal_options_);
        future_gripper_result_ = promise_.get_future();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        std::future_status status = exec_future.wait_for(std::chrono::seconds(10));
        switch (status)
        {
            case std::future_status::ready:
            {
                auto goal_handle = exec_future.get();
                if (goal_handle->is_result_aware())
                {
                    auto res = future_gripper_result_.get();
                    if (res)
                    {
                        RCLCPP_INFO(node_->get_logger(), "Gripper action complete");
                        return BT::NodeStatus::SUCCESS;
                    }
                    else
                    {
                        RCLCPP_ERROR(node_->get_logger(), "Gripper action failed");
                        return BT::NodeStatus::FAILURE;
                    }
                }
            }
            case std::future_status::timeout:
            {
                // RCLCPP_INFO(node_->get_logger(), "Execution running");
                return BT::NodeStatus::RUNNING;
            }
            case std::future_status::deferred:
            {
                RCLCPP_ERROR(node_->get_logger(), "Execution deferred");
                return BT::NodeStatus::FAILURE;
            }
            default:
                return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override { RCLCPP_INFO(node_->get_logger(), "Execution halted"); }

  protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<GripperCommand>::SharedPtr client_;
    rclcpp_action::Client<GripperCommand>::SendGoalOptions goal_options_;
    std::string gripper_action_;

    std::promise<bool> promise_;
    std::shared_future<bool> future_gripper_result_;

    std::shared_future<std::shared_ptr<GoalHandleGripperCommand>> exec_future;

  private:
    void feedback_callback(GoalHandleGripperCommand::SharedPtr,
                           const std::shared_ptr<const GripperCommand::Feedback> feedback)
    {
        RCLCPP_INFO(node_->get_logger(), "Received feedback: %f", feedback->position);
    }

    void goal_response_callback(const GoalHandleGripperCommand::SharedPtr &goal_handle)
    {
        using namespace std::placeholders;
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const GoalHandleGripperCommand::WrappedResult &result)
    {
        using namespace std::placeholders;
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
                promise_.set_value(true);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
                promise_.set_value(false);
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
                promise_.set_value(false);
                return;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
                promise_.set_value(false);
                return;
        }
        RCLCPP_INFO(node_->get_logger(), "Result received");
    }
};

#endif // GRIPPER_NODE_HPP
