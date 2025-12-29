#include "segwayrmp/rmp401_driver_node.hpp"

namespace segwayrmp
{

  Rmp401DriverNode::Rmp401DriverNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("rmp401_driver_node", options)
  {
    std::string serial_device;
    declare_parameter<std::string>("serial_full_name", "ttyUSB0");
    get_parameter("serial_full_name", serial_device);

    RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_device.c_str());

    set_smart_car_serial(serial_device.c_str());
    set_comu_interface(comu_serial);

    // 이 시점에는 shared_from_this() 불가능하므로, 타이머를 통해 지연 초기화
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Rmp401DriverNode::setup, this));
  }

  void Rmp401DriverNode::setup()
  {
    // shared_from_this() 사용 안전한 시점
    try
    {
      auto node_ptr = this->shared_from_this();
      chassis_ = std::make_shared<robot::Chassis>(node_ptr);
      RCLCPP_INFO(this->get_logger(), "Rmp401DriverNode initialized successfully.");
      init_timer_->cancel(); // 타이머 종료
    }
    catch (const std::bad_weak_ptr &e)
    {
      RCLCPP_FATAL(this->get_logger(), "shared_from_this() failed: %s", e.what());
    }
  }

} // namespace segwayrmp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(segwayrmp::Rmp401DriverNode)
