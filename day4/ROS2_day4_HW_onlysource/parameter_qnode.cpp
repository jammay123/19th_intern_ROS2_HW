#include "../include/control_cam_params/parameter_qnode.hpp"

ParameterQNode::ParameterQNode() : QThread()
{
  node = rclcpp::Node::make_shared("parameter_qnode");

  node->declare_parameter("hue_upper", 180);
  node->declare_parameter("hue_lower", 0);
  node->declare_parameter("saturation_upper", 255);
  node->declare_parameter("saturation_lower", 0);
  node->declare_parameter("value_upper", 255);
  node->declare_parameter("value_lower", 0);

  node->declare_parameter("image_width", 640);
  node->declare_parameter("image_height", 360);

  node->declare_parameter("erode_cnt", 0);
  node->declare_parameter("dilate_cnt", 0);

  node->declare_parameter("canny_min", 100);
  node->declare_parameter("canny_max", 200);

  node->declare_parameter("roi_p1_x", 160);
  node->declare_parameter("roi_p1_y", 90);
  node->declare_parameter("roi_p2_x", 480);
  node->declare_parameter("roi_p2_y", 90);
  node->declare_parameter("roi_p3_x", 480);
  node->declare_parameter("roi_p3_y", 270);
  node->declare_parameter("roi_p4_x", 160);
  node->declare_parameter("roi_p4_y", 270);

  this->start();
}

ParameterQNode::~ParameterQNode()
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void ParameterQNode::run()
{
  rclcpp::spin(node);
}

void ParameterQNode::setHueUpper(int value)
{
  node->set_parameters({rclcpp::Parameter("hue_upper", value)});
  RCLCPP_INFO(node->get_logger(), "Hue Upper set to: %d", value);
}

void ParameterQNode::setHueLower(int value)
{
  node->set_parameters({rclcpp::Parameter("hue_lower", value)});
  RCLCPP_INFO(node->get_logger(), "Hue Lower set to: %d", value);
}

void ParameterQNode::setSatrUpper(int value)
{
  node->set_parameters({rclcpp::Parameter("saturation_upper", value)});
  RCLCPP_INFO(node->get_logger(), "Saturation Upper set to: %d", value);
}

void ParameterQNode::setSatrLower(int value)
{
  node->set_parameters({rclcpp::Parameter("saturation_lower", value)});
  RCLCPP_INFO(node->get_logger(), "Saturation Lower set to: %d", value);
}

void ParameterQNode::setValUpper(int value)
{
  node->set_parameters({rclcpp::Parameter("value_upper", value)});
  RCLCPP_INFO(node->get_logger(), "Value Upper set to: %d", value);
}

void ParameterQNode::setValLower(int value)
{
  node->set_parameters({rclcpp::Parameter("value_lower", value)});
  RCLCPP_INFO(node->get_logger(), "Value Lower set to: %d", value);
}

void ParameterQNode::setImgWidth(int value)
{
  node->set_parameters({rclcpp::Parameter("image_width", value)});
  RCLCPP_INFO(node->get_logger(), "Image Width set to: %d", value);
}

void ParameterQNode::setImgHeight(int value)
{
  node->set_parameters({rclcpp::Parameter("image_height", value)});
  RCLCPP_INFO(node->get_logger(), "Image Height set to: %d", value);
}

void ParameterQNode::setErodeCnt(int value)
{
  node->set_parameters({rclcpp::Parameter("erode_cnt", value)});
  RCLCPP_INFO(node->get_logger(), "Erode Count set to: %d", value);
}

void ParameterQNode::setDilateCnt(int value)
{
  node->set_parameters({rclcpp::Parameter("dilate_cnt", value)});
  RCLCPP_INFO(node->get_logger(), "Dilate Count set to: %d", value);
}

void ParameterQNode::setCannyMin(int value)
{
  node->set_parameters({rclcpp::Parameter("canny_min", value)});
  RCLCPP_INFO(node->get_logger(), "Canny Min set to: %d", value);
}

void ParameterQNode::setCannyMax(int value)
{
  node->set_parameters({rclcpp::Parameter("canny_max", value)});
  RCLCPP_INFO(node->get_logger(), "Canny Max set to: %d", value);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ParameterQNode::setRoiP1X(int value) {
    node->set_parameters({rclcpp::Parameter("roi_p1_x", value)});
}

void ParameterQNode::setRoiP1Y(int value) {
    node->set_parameters({rclcpp::Parameter("roi_p1_y", value)});
}

void ParameterQNode::setRoiP2X(int value) {
    node->set_parameters({rclcpp::Parameter("roi_p2_x", value)});
}

void ParameterQNode::setRoiP2Y(int value) {
    node->set_parameters({rclcpp::Parameter("roi_p2_y", value)});
}

void ParameterQNode::setRoiP3X(int value) {
    node->set_parameters({rclcpp::Parameter("roi_p3_x", value)});
}

void ParameterQNode::setRoiP3Y(int value) {
    node->set_parameters({rclcpp::Parameter("roi_p3_y", value)});
}

void ParameterQNode::setRoiP4X(int value) {
    node->set_parameters({rclcpp::Parameter("roi_p4_x", value)});
}

void ParameterQNode::setRoiP4Y(int value) {
    node->set_parameters({rclcpp::Parameter("roi_p4_y", value)});
}
