#include "../include/cam_params_pkg/parameter_img_node.hpp"

ParameterImgNode::ParameterImgNode(rclcpp::Node::SharedPtr node, QObject* parent)
  : QThread(parent), node_(node)
{
  // AsyncParametersClient 초기화
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "image_processing_node");
}

void ParameterImgNode::run()
{
  // 비동기 클라이언트 노드 실행
  rclcpp::spin(node_);
}

void ParameterImgNode::setParameter(const std::string& name, int value)
{
  // 파라미터 설정 요청
  auto parameter = rclcpp::Parameter(name, value);
  parameters_client_->set_parameters({parameter});
}

ParameterImgNode::~ParameterImgNode() {}
