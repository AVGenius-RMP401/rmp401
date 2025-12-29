#ifndef SEGWAYRMP__RMP401_DRIVER_NODE_HPP_
#define SEGWAYRMP__RMP401_DRIVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "segwayrmp/robot.h"

namespace segwayrmp
{

  class Rmp401DriverNode : public rclcpp::Node
  {
  public:
    explicit Rmp401DriverNode(const rclcpp::NodeOptions & options);
  
  private:
    void setup();
    std::shared_ptr<robot::Chassis> chassis_;
    rclcpp::TimerBase::SharedPtr init_timer_;
  };
  
}  // namespace segwayrmp

#endif  // SEGWAYRMP__RMP401_DRIVER_NODE_HPP_
