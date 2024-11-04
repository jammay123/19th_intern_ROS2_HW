#include "../include/control_cam_params/qnode.hpp"

QNode::QNode()
{
  node = rclcpp::Node::make_shared("control_cam_params");
  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {

  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}
