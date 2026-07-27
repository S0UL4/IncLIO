// Custom main using MultiThreadedExecutor so that IMU and LiDAR
// callback groups can run in parallel.

#include <rclcpp/rclcpp.hpp>
#include "ros2_wrapper/lio_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<inclio_ros2::LioNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
