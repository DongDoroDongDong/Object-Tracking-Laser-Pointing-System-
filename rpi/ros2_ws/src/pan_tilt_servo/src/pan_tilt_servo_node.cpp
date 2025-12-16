#include "pan_tilt_servo/pan_tilt_servo_node.hpp"
#include <algorithm>
#include <cmath>

PanTiltServoNode::PanTiltServoNode()
: Node("pan_tilt_servo_node"),
  pi_(-1),
  target_pan_deg_(0.0),
  target_tilt_deg_(0.0),
  current_pan_deg_(0.0),
  current_tilt_deg_(0.0)
{
  pan_gpio_  = this->declare_parameter<int>("pan_gpio", 18);
  tilt_gpio_ = this->declare_parameter<int>("tilt_gpio", 19);

  min_us_ = this->declare_parameter<int>("min_us", 500);
  max_us_ = this->declare_parameter<int>("max_us", 2500);

  pan_deg_min_  = this->declare_parameter<double>("pan_deg_min", -90.0);
  pan_deg_max_  = this->declare_parameter<double>("pan_deg_max",  90.0);
  tilt_deg_min_ = this->declare_parameter<double>("tilt_deg_min", -45.0);
  tilt_deg_max_ = this->declare_parameter<double>("tilt_deg_max",  45.0);

  alpha_ = this->declare_parameter<double>("alpha", 0.25);
  int apply_rate_hz = this->declare_parameter<int>("apply_rate_hz", 50);

  pi_ = pigpio_start(nullptr, nullptr);
  if (pi_ < 0) {
    RCLCPP_FATAL(this->get_logger(),
                 "pigpio_start 실패. pigpiod가 실행 중인지 확인하세요. (sudo pigpiod)");
    throw std::runtime_error("pigpio_start failed");
  }

  set_mode(pi_, pan_gpio_, PI_OUTPUT);
  set_mode(pi_, tilt_gpio_, PI_OUTPUT);

  applyServoPulse();

  sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "pan_tilt_cmd",
    rclcpp::QoS(10),
    std::bind(&PanTiltServoNode::cmdCallback, this, std::placeholders::_1)
  );

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / apply_rate_hz)),
    std::bind(&PanTiltServoNode::applyServoPulse, this)
  );

  RCLCPP_INFO(this->get_logger(),
              "PanTiltServoNode(데몬 모드) 시작. pan_gpio=%d tilt_gpio=%d",
              pan_gpio_, tilt_gpio_);
}

PanTiltServoNode::~PanTiltServoNode()
{
  set_servo_pulsewidth(pi_, pan_gpio_, 0);
  set_servo_pulsewidth(pi_, tilt_gpio_, 0);
  pigpio_stop(pi_);
}

double PanTiltServoNode::clamp(double v, double vmin, double vmax) const {
  return std::max(vmin, std::min(v, vmax));
}

int PanTiltServoNode::angleDegToPulseUS(double deg, double deg_min, double deg_max) const
{
  deg = clamp(deg, deg_min, deg_max);
  double t = (deg - deg_min) / (deg_max - deg_min);  
  double us = min_us_ + t * (max_us_ - min_us_);
  return static_cast<int>(std::round(us));
}

void PanTiltServoNode::cmdCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) {
    RCLCPP_WARN(this->get_logger(),
                "pan_tilt_cmd 토픽은 2개의 float가 필요합니다: [pan_deg, tilt_deg]");
    return;
  }

  target_pan_deg_  = msg->data[0];
  target_tilt_deg_ = msg->data[1];
}

void PanTiltServoNode::applyServoPulse()
{
  current_pan_deg_  = (1.0 - alpha_) * current_pan_deg_  + alpha_ * target_pan_deg_;
  current_tilt_deg_ = (1.0 - alpha_) * current_tilt_deg_ + alpha_ * target_tilt_deg_;

  int pan_us  = angleDegToPulseUS(current_pan_deg_,  pan_deg_min_,  pan_deg_max_);
  int tilt_us = angleDegToPulseUS(current_tilt_deg_, tilt_deg_min_, tilt_deg_max_);

  set_servo_pulsewidth(pi_, pan_gpio_, pan_us);
  set_servo_pulsewidth(pi_, tilt_gpio_, tilt_us);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PanTiltServoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

