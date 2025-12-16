#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <pigpiod_if2.h>

class PanTiltServoNode : public rclcpp::Node {
public:
  PanTiltServoNode();
  ~PanTiltServoNode();

private:
  void cmdCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void applyServoPulse();

  int angleDegToPulseUS(double deg, double deg_min, double deg_max) const;
  double clamp(double v, double vmin, double vmax) const;

  int pi_;

  int pan_gpio_;
  int tilt_gpio_;

  int min_us_;
  int max_us_;

  double pan_deg_min_;
  double pan_deg_max_;
  double tilt_deg_min_;
  double tilt_deg_max_;

  double alpha_;

  double target_pan_deg_;
  double target_tilt_deg_;
  double current_pan_deg_;
  double current_tilt_deg_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

