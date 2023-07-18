#include "rclcpp/rclcpp.hpp"
#include "any_robot_pick_n_place/bt_manipulation_manager.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("any_robot_pick_n_place_node", options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_shared<std::thread>([&executor, &node]() 
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