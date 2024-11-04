#ifndef CONTROL_CAM_PARAMS_PARAMETER_QNODE_HPP_
#define CONTROL_CAM_PARAMS_PARAMETER_QNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <QThread>

class ParameterQNode : public QThread
{
  Q_OBJECT
public:
  ParameterQNode();
  ~ParameterQNode();

  void setHueUpper(int value);
  void setHueLower(int value);
  void setSatrUpper(int value);
  void setSatrLower(int value);
  void setValUpper(int value);
  void setValLower(int value);
  void setImgWidth(int value);
  void setImgHeight(int value);
  void setErodeCnt(int value);
  void setDilateCnt(int value);
  void setCannyMin(int value);
  void setCannyMax(int value);
  void setRoiP1X(int value);
  void setRoiP1Y(int value);
  void setRoiP2X(int value);
  void setRoiP2Y(int value);
  void setRoiP3X(int value);
  void setRoiP3Y(int value);
  void setRoiP4X(int value);
  void setRoiP4Y(int value);

protected:
  void run() override;

private:
  std::shared_ptr<rclcpp::Node> node;
};

#endif  // CONTROL_CAM_PARAMS_PARAMETER_QNODE_HPP_
