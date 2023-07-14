#ifndef EXECUTE_MOTION_NODE_HPP
#define EXECUTE_MOTION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"
#include "geometry_msgs/msg/pose.hpp"

#include <behaviortree_cpp/bt_factory.h>

class ExecuteMotionNode: public BT::StatefulActionNode
{
    protected:
        rclcpp::Node::SharedPtr node_;
        moveit_cpp::MoveItCppPtr moveit_cpp_;
        moveit_cpp::PlanningComponentPtr planning_component_;
        std::string planning_goup_name_;
        std::future<moveit_controller_manager::ExecutionStatus> exe_future_;

    public:
        ExecuteMotionNode(const std::string &name, const BT::NodeConfiguration &config) : BT::StatefulActionNode(name, config)
        {
            node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
            moveit_cpp_ = config.blackboard->get<moveit_cpp::MoveItCppPtr>("moveit_cpp");
            planning_component_ = config.blackboard->get<moveit_cpp::PlanningComponentPtr>("planning_component");
            planning_goup_name_ = config.blackboard->get<std::string>("planning_group_name");
        }

        static BT::PortsList providedPorts()
        {
            const char *op_description = "Execute Action Description";
            return {
                BT::InputPort<std::string>("Operation", op_description),
                BT::InputPort<robot_trajectory::RobotTrajectoryPtr>("trajectory"),
            };
        }

        BT::NodeStatus onStart() override
        {
            RCLCPP_INFO(node_->get_logger(), "Executing %s", getInput<std::string>("Operation").value().c_str());
            
            if(!getInput<robot_trajectory::RobotTrajectoryPtr>("trajectory").has_value())
            {
                RCLCPP_ERROR(node_->get_logger(), "Trajectory not set. Please set the Trajectory to be executed. Exception thrown: %s", getInput<robot_trajectory::RobotTrajectoryPtr>("trajectory").error().c_str());
                return BT::NodeStatus::FAILURE;
            }

            robot_trajectory::RobotTrajectoryPtr trajectory = getInput<robot_trajectory::RobotTrajectoryPtr>("trajectory").value();

            exe_future_ = std::async(std::launch::async, [this, &trajectory](){
                auto result = this->moveit_cpp_->execute(planning_goup_name_, trajectory);
                RCLCPP_INFO(node_->get_logger(), "Execution result: %d", result);
                return result;
            });
            BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override
        {
            std::future_status status = exe_future_.wait_for(std::chrono::milliseconds(1));
            switch(status)
            {
                case std::future_status::ready:
                {
                    auto result = exe_future_.get();
                    if(result == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
                    {
                        RCLCPP_INFO(node_->get_logger(), "Execution succeeded");
                        return BT::NodeStatus::SUCCESS;
                    }
                    else
                    {
                        RCLCPP_ERROR(node_->get_logger(), "Execution failed");
                        return BT::NodeStatus::FAILURE;
                    }
                }
                case std::future_status::timeout:
                {
                    RCLCPP_INFO(node_->get_logger(), "Execution running");
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

        void onHalted() override
        {
            RCLCPP_INFO(node_->get_logger(), "Execution halted");
            moveit_cpp_->getTrajectoryExecutionManager()->stopExecution();
        }
};

#endif // EXECUTE_MOTION_NODE_HPP