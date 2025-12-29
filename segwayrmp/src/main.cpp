#include "rclcpp/rclcpp.hpp"
#include "segwayrmp/rmp401_driver_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;  // 수정: 옵션에서 자동 선언 비활성화
    auto node = std::make_shared<segwayrmp::Rmp401DriverNode>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
