#include <memory>
#include <string>
#include <algorithm>
#include <cmath>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "sensor_msgs/msg/image.hpp"

class PanTiltTracker : public rclcpp::Node
{
public:
  PanTiltTracker()
  : Node("pan_tilt_tracker"),
    image_width_(320),
    image_height_(240),
    hfov_deg_(68.3),
    vfov_deg_(53.9),
    hfov_rad_(0.0),
    vfov_rad_(0.0),

    control_mode_("absolute_xy"),
    pan_sign_(1.0),
    tilt_sign_(1.0),
    camera_rotated_180_(true),

    target_class_id_("0"),

    base_pan_deg_(0.0),
    base_tilt_deg_(0.0),

    kp_pan_(2.0),
    kp_tilt_(2.0),
    k_gain_pan_(0.8),
    k_gain_tilt_(0.8),

    use_directional_damping_(true),
    k_gain_pan_dir8_(8, 1.0),
    k_gain_tilt_dir8_(8, 1.0),

    neg_pan_threshold_deg_(-1.0),
    neg_pan_damping_mult_(2.0),
    pos_pan_damping_mult_(1.0),

    edge_damping_pan_(0.0),
    edge_damping_tilt_(0.0),

    pan_min_deg_(-90.0),
    pan_max_deg_(90.0),
    tilt_min_deg_(-20.0),
    tilt_max_deg_(20.0),

    use_smoothing_(true),
    smoothing_factor_(0.35),
    max_pan_speed_deg_per_sec_(120.0),
    max_tilt_speed_deg_per_sec_(120.0),

    pan_offset_deg_(0.0),
    tilt_offset_deg_(0.0),

    target_distance_m_(0.5),
    dist_points_(),
    pan_offset_points_(),
    tilt_offset_points_(),

    use_image_topic_size_(true),
    image_topic_for_size_("/inference/image_raw"),

    prev_pan_cmd_deg_(0.0),
    prev_tilt_cmd_deg_(0.0),
    prev_cmd_initialized_(false)
  {
    last_cmd_time_ = this->now();

    image_width_  = this->declare_parameter<int>("image_width", image_width_);
    image_height_ = this->declare_parameter<int>("image_height", image_height_);

    hfov_deg_ = this->declare_parameter<double>("hfov_deg", hfov_deg_);
    vfov_deg_ = this->declare_parameter<double>("vfov_deg", vfov_deg_);

    control_mode_ = this->declare_parameter<std::string>("control_mode", control_mode_);
    pan_sign_  = this->declare_parameter<double>("pan_sign", pan_sign_);
    tilt_sign_ = this->declare_parameter<double>("tilt_sign", tilt_sign_);

    camera_rotated_180_ = this->declare_parameter<bool>("camera_rotated_180", camera_rotated_180_);
    target_class_id_    = this->declare_parameter<std::string>("target_class_id", target_class_id_);

    base_pan_deg_  = this->declare_parameter<double>("base_pan_deg", base_pan_deg_);
    base_tilt_deg_ = this->declare_parameter<double>("base_tilt_deg", base_tilt_deg_);

    kp_pan_  = this->declare_parameter<double>("kp_pan", kp_pan_);
    kp_tilt_ = this->declare_parameter<double>("kp_tilt", kp_tilt_);

    k_gain_pan_  = std::max(0.0, this->declare_parameter<double>("k_gain_pan", k_gain_pan_));
    k_gain_tilt_ = std::max(0.0, this->declare_parameter<double>("k_gain_tilt", k_gain_tilt_));

    use_directional_damping_ =
      this->declare_parameter<bool>("use_directional_damping", use_directional_damping_);

    k_gain_pan_dir8_ = this->declare_parameter<std::vector<double>>(
      "k_gain_pan_dir8", k_gain_pan_dir8_);
    k_gain_tilt_dir8_ = this->declare_parameter<std::vector<double>>(
      "k_gain_tilt_dir8", k_gain_tilt_dir8_);

    sanitizeDir8(k_gain_pan_dir8_);
    sanitizeDir8(k_gain_tilt_dir8_);

    neg_pan_threshold_deg_ = this->declare_parameter<double>("neg_pan_threshold_deg", neg_pan_threshold_deg_);
    neg_pan_damping_mult_  = std::max(0.0, this->declare_parameter<double>("neg_pan_damping_mult", neg_pan_damping_mult_));
    pos_pan_damping_mult_  = std::max(0.0, this->declare_parameter<double>("pos_pan_damping_mult", pos_pan_damping_mult_));

    edge_damping_pan_  = std::max(0.0, this->declare_parameter<double>("edge_damping_pan", edge_damping_pan_));
    edge_damping_tilt_ = std::max(0.0, this->declare_parameter<double>("edge_damping_tilt", edge_damping_tilt_));

    pan_min_deg_  = this->declare_parameter<double>("pan_min_deg", pan_min_deg_);
    pan_max_deg_  = this->declare_parameter<double>("pan_max_deg", pan_max_deg_);
    tilt_min_deg_ = this->declare_parameter<double>("tilt_min_deg", tilt_min_deg_);
    tilt_max_deg_ = this->declare_parameter<double>("tilt_max_deg", tilt_max_deg_);

    use_smoothing_ = this->declare_parameter<bool>("use_smoothing", use_smoothing_);
    smoothing_factor_ = std::clamp(this->declare_parameter<double>("smoothing_factor", smoothing_factor_), 0.0, 1.0);
    max_pan_speed_deg_per_sec_  = this->declare_parameter<double>("max_pan_speed_deg_per_sec", max_pan_speed_deg_per_sec_);
    max_tilt_speed_deg_per_sec_ = this->declare_parameter<double>("max_tilt_speed_deg_per_sec", max_tilt_speed_deg_per_sec_);

    pan_offset_deg_  = this->declare_parameter<double>("pan_offset_deg", pan_offset_deg_);
    tilt_offset_deg_ = this->declare_parameter<double>("tilt_offset_deg", tilt_offset_deg_);

    target_distance_m_ = this->declare_parameter<double>("target_distance_m", target_distance_m_);

    dist_points_ = this->declare_parameter<std::vector<double>>("dist_points", std::vector<double>{});
    pan_offset_points_ = this->declare_parameter<std::vector<double>>("pan_offset_points", std::vector<double>{});
    tilt_offset_points_ = this->declare_parameter<std::vector<double>>("tilt_offset_points", std::vector<double>{});

    use_image_topic_size_ = this->declare_parameter<bool>("use_image_topic_size", use_image_topic_size_);
    image_topic_for_size_ = this->declare_parameter<std::string>("image_topic_for_size", image_topic_for_size_);

    std::string detection_topic =
      this->declare_parameter<std::string>("detection_topic", "/detections");
    std::string pan_tilt_cmd_topic =
      this->declare_parameter<std::string>("pan_tilt_cmd_topic", "/pan_tilt_cmd");

    updateFovRad();

    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PanTiltTracker::onParamChange, this, std::placeholders::_1));

    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      detection_topic, 10,
      std::bind(&PanTiltTracker::detectionCallback, this, std::placeholders::_1));

    pan_tilt_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      pan_tilt_cmd_topic, 10);

    if (use_image_topic_size_) {
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_for_size_, 1,
        std::bind(&PanTiltTracker::imageSizeCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Image size source: %s", image_topic_for_size_.c_str());
    }

    RCLCPP_INFO(this->get_logger(),
      "Ready. mode=%s hfov=%.2f vfov=%.2f | dir8=%s | neg_pan(th=%.2f, mult=%.2f, pos_mult=%.2f)",
      control_mode_.c_str(), hfov_deg_, vfov_deg_,
      use_directional_damping_ ? "ON" : "OFF",
      neg_pan_threshold_deg_, neg_pan_damping_mult_, pos_pan_damping_mult_);
  }

private:
  static void sanitizeDir8(std::vector<double>& v)
  {
    if (v.size() != 8) v.assign(8, 1.0);
    for (auto& x : v) {
      if (!std::isfinite(x)) x = 1.0;
      if (x < 0.0) x = 0.0;
    }
  }

  static int dir8Index(double x_norm, double y_norm)
  {
    constexpr double RAD2DEG = 180.0 / 3.14159265358979323846;
    double deg = std::atan2(y_norm, x_norm) * RAD2DEG; 
    if (deg < 0.0) deg += 360.0;
    int idx = static_cast<int>(std::floor((deg + 22.5) / 45.0)) % 8;
    return idx;
  }

  rcl_interfaces::msg::SetParametersResult
  onParamChange(const std::vector<rclcpp::Parameter>& params)
  {
    bool fov_changed = false;

    for (const auto& p : params) {
      const auto& n = p.get_name();

      if (n == "hfov_deg") { hfov_deg_ = p.as_double(); fov_changed = true; }
      else if (n == "vfov_deg") { vfov_deg_ = p.as_double(); fov_changed = true; }

      else if (n == "control_mode") { control_mode_ = p.as_string(); }
      else if (n == "pan_sign") { pan_sign_ = p.as_double(); }
      else if (n == "tilt_sign") { tilt_sign_ = p.as_double(); }

      else if (n == "camera_rotated_180") { camera_rotated_180_ = p.as_bool(); }
      else if (n == "target_class_id") { target_class_id_ = p.as_string(); }

      else if (n == "base_pan_deg") { base_pan_deg_ = p.as_double(); }
      else if (n == "base_tilt_deg") { base_tilt_deg_ = p.as_double(); }

      else if (n == "kp_pan") { kp_pan_ = p.as_double(); }
      else if (n == "kp_tilt") { kp_tilt_ = p.as_double(); }
      else if (n == "k_gain_pan") { k_gain_pan_ = std::max(0.0, p.as_double()); }
      else if (n == "k_gain_tilt") { k_gain_tilt_ = std::max(0.0, p.as_double()); }

      else if (n == "use_directional_damping") { use_directional_damping_ = p.as_bool(); }
      else if (n == "k_gain_pan_dir8") { k_gain_pan_dir8_ = p.as_double_array(); sanitizeDir8(k_gain_pan_dir8_); }
      else if (n == "k_gain_tilt_dir8") { k_gain_tilt_dir8_ = p.as_double_array(); sanitizeDir8(k_gain_tilt_dir8_); }

      else if (n == "neg_pan_threshold_deg") { neg_pan_threshold_deg_ = p.as_double(); }
      else if (n == "neg_pan_damping_mult") { neg_pan_damping_mult_ = std::max(0.0, p.as_double()); }
      else if (n == "pos_pan_damping_mult") { pos_pan_damping_mult_ = std::max(0.0, p.as_double()); }

      else if (n == "edge_damping_pan") { edge_damping_pan_ = std::max(0.0, p.as_double()); }
      else if (n == "edge_damping_tilt") { edge_damping_tilt_ = std::max(0.0, p.as_double()); }

      else if (n == "use_smoothing") { use_smoothing_ = p.as_bool(); }
      else if (n == "smoothing_factor") { smoothing_factor_ = std::clamp(p.as_double(), 0.0, 1.0); }
      else if (n == "max_pan_speed_deg_per_sec") { max_pan_speed_deg_per_sec_ = p.as_double(); }
      else if (n == "max_tilt_speed_deg_per_sec") { max_tilt_speed_deg_per_sec_ = p.as_double(); }

      else if (n == "pan_offset_deg") { pan_offset_deg_ = p.as_double(); }
      else if (n == "tilt_offset_deg") { tilt_offset_deg_ = p.as_double(); }

      else if (n == "target_distance_m") { target_distance_m_ = p.as_double(); }
      else if (n == "dist_points") { dist_points_ = p.as_double_array(); }
      else if (n == "pan_offset_points") { pan_offset_points_ = p.as_double_array(); }
      else if (n == "tilt_offset_points") { tilt_offset_points_ = p.as_double_array(); }
    }

    if (fov_changed) updateFovRad();

    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    res.reason = "ok";
    return res;
  }

  void updateFovRad()
  {
    constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
    hfov_rad_ = hfov_deg_ * DEG2RAD;
    vfov_rad_ = vfov_deg_ * DEG2RAD;
  }

  void imageSizeCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const int w = static_cast<int>(msg->width);
    const int h = static_cast<int>(msg->height);
    if (w <= 0 || h <= 0) return;

    std::lock_guard<std::mutex> lock(img_size_mutex_);
    if (w != image_width_ || h != image_height_) {
      image_width_ = w;
      image_height_ = h;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Updated image size: %dx%d", image_width_, image_height_);
    }
  }

  double lookupDistanceOffset(double d,
                              const std::vector<double>& dist_points,
                              const std::vector<double>& offset_points) const
  {
    if (dist_points.size() != offset_points.size() || dist_points.empty()) return 0.0;
    if (d <= dist_points.front()) return offset_points.front();
    if (d >= dist_points.back())  return offset_points.back();

    for (size_t i = 0; i + 1 < dist_points.size(); ++i) {
      const double d0 = dist_points[i], d1 = dist_points[i + 1];
      if (d >= d0 && d <= d1) {
        const double o0 = offset_points[i], o1 = offset_points[i + 1];
        const double t = (d - d0) / (d1 - d0);
        return o0 + t * (o1 - o0);
      }
    }
    return 0.0;
  }

  static double clampToRange(double v, double mn, double mx)
  {
    if (v < mn) return mn;
    if (v > mx) return mx;
    return v;
  }

  void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    if (msg->detections.empty()) return;

    const vision_msgs::msg::Detection2D* best = nullptr;
    double best_score = 0.0;

    for (const auto& det : msg->detections) {
      for (const auto& res : det.results) {
        if (res.hypothesis.class_id == target_class_id_) {
          if (res.hypothesis.score > best_score) {
            best_score = res.hypothesis.score;
            best = &det;
          }
        }
      }
    }
    if (!best) return;

    const double cx = best->bbox.center.position.x;
    const double cy = best->bbox.center.position.y;

    int w, h;
    {
      std::lock_guard<std::mutex> lock(img_size_mutex_);
      w = image_width_;
      h = image_height_;
    }
    if (w <= 0 || h <= 0) return;

    const double frame_cx = w / 2.0;
    const double frame_cy = h / 2.0;

    double x_norm = (cx - frame_cx) / frame_cx;
    double y_norm = (cy - frame_cy) / frame_cy;

    x_norm = std::clamp(x_norm, -2.0, 2.0);
    y_norm = std::clamp(y_norm, -2.0, 2.0);

    if (camera_rotated_180_) { x_norm *= -1.0; y_norm *= -1.0; }

    const double pan_err_rad  = -std::atan(x_norm * std::tan(hfov_rad_ / 2.0));
    const double tilt_err_rad =  std::atan(y_norm * std::tan(vfov_rad_ / 2.0));

    constexpr double RAD2DEG = 180.0 / 3.14159265358979323846;
    double pan_err_deg  = pan_err_rad  * RAD2DEG;
    double tilt_err_deg = tilt_err_rad * RAD2DEG;

    pan_err_deg  *= pan_sign_;
    tilt_err_deg *= tilt_sign_;

    const double r_pan  = std::min(1.0, std::abs(x_norm));
    const double r_tilt = std::min(1.0, std::abs(y_norm));

    const double edge_scale_pan  = 1.0 / (1.0 + edge_damping_pan_  * r_pan  * r_pan);
    const double edge_scale_tilt = 1.0 / (1.0 + edge_damping_tilt_ * r_tilt * r_tilt);
    pan_err_deg  *= edge_scale_pan;
    tilt_err_deg *= edge_scale_tilt;

    const int idx8 = dir8Index(x_norm, y_norm);

    double kpan  = k_gain_pan_;
    double ktilt = k_gain_tilt_;
    if (use_directional_damping_) {
      kpan  *= k_gain_pan_dir8_[idx8];
      ktilt *= k_gain_tilt_dir8_[idx8];
    }

    double pan_ref = 0.0;
    if (control_mode_ == "absolute_xy") {
      pan_ref = base_pan_deg_;
    } else {
      pan_ref = prev_cmd_initialized_ ? prev_pan_cmd_deg_ : 0.0;
    }

    if (pan_ref <= neg_pan_threshold_deg_) {
      kpan *= neg_pan_damping_mult_;
    } else {
      kpan *= pos_pan_damping_mult_;
    }

    const double kp_pan_eff  = kp_pan_  / (1.0 + kpan  * r_pan  * r_pan);
    const double kp_tilt_eff = kp_tilt_ / (1.0 + ktilt * r_tilt * r_tilt);

    const double delta_pan_deg  = kp_pan_eff  * pan_err_deg;
    const double delta_tilt_deg = kp_tilt_eff * tilt_err_deg;

    const double pan_dist_off  = lookupDistanceOffset(target_distance_m_, dist_points_, pan_offset_points_);
    const double tilt_dist_off = lookupDistanceOffset(target_distance_m_, dist_points_, tilt_offset_points_);

    double target_pan_cmd  = 0.0;
    double target_tilt_cmd = 0.0;

    if (control_mode_ == "absolute_xy") {
      target_pan_cmd  = base_pan_deg_  + delta_pan_deg  + pan_offset_deg_  + pan_dist_off;
      target_tilt_cmd = base_tilt_deg_ + delta_tilt_deg + tilt_offset_deg_ + tilt_dist_off;
    } else {
      const double base_pan  = prev_cmd_initialized_ ? prev_pan_cmd_deg_  : 0.0;
      const double base_tilt = prev_cmd_initialized_ ? prev_tilt_cmd_deg_ : 0.0;
      target_pan_cmd  = base_pan  + delta_pan_deg  + pan_offset_deg_  + pan_dist_off;
      target_tilt_cmd = base_tilt + delta_tilt_deg + tilt_offset_deg_ + tilt_dist_off;
    }

    target_pan_cmd  = clampToRange(target_pan_cmd,  pan_min_deg_,  pan_max_deg_);
    target_tilt_cmd = clampToRange(target_tilt_cmd, tilt_min_deg_, tilt_max_deg_);

    const rclcpp::Time now = this->now();
    double dt = (now - last_cmd_time_).seconds();
    if (dt <= 0.0) dt = 1.0 / 30.0;

    double pan_out  = target_pan_cmd;
    double tilt_out = target_tilt_cmd;

    if (use_smoothing_) {
      if (!prev_cmd_initialized_) {
        prev_pan_cmd_deg_  = target_pan_cmd;
        prev_tilt_cmd_deg_ = target_tilt_cmd;
        prev_cmd_initialized_ = true;
      }

      const double smoothed_pan  =
        prev_pan_cmd_deg_ + smoothing_factor_ * (target_pan_cmd - prev_pan_cmd_deg_);
      const double smoothed_tilt =
        prev_tilt_cmd_deg_ + smoothing_factor_ * (target_tilt_cmd - prev_tilt_cmd_deg_);

      const double max_pan_delta  = max_pan_speed_deg_per_sec_  * dt;
      const double max_tilt_delta = max_tilt_speed_deg_per_sec_ * dt;

      double pan_delta  = smoothed_pan  - prev_pan_cmd_deg_;
      double tilt_delta = smoothed_tilt - prev_tilt_cmd_deg_;

      pan_delta  = clampToRange(pan_delta,  -max_pan_delta,  max_pan_delta);
      tilt_delta = clampToRange(tilt_delta, -max_tilt_delta, max_tilt_delta);

      pan_out  = prev_pan_cmd_deg_  + pan_delta;
      tilt_out = prev_tilt_cmd_deg_ + tilt_delta;
    }

    pan_out  = clampToRange(pan_out,  pan_min_deg_,  pan_max_deg_);
    tilt_out = clampToRange(tilt_out, tilt_min_deg_, tilt_max_deg_);

    prev_pan_cmd_deg_ = pan_out;
    prev_tilt_cmd_deg_ = tilt_out;
    last_cmd_time_ = now;
    prev_cmd_initialized_ = true;

    std_msgs::msg::Float32MultiArray cmd;
    cmd.data.resize(2);
    cmd.data[0] = static_cast<float>(pan_out);
    cmd.data[1] = static_cast<float>(tilt_out);
    pan_tilt_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "dir8=%d x=%.3f y=%.3f r(p=%.2f,t=%.2f) kpan=%.2f (ref=%.2f) kp_eff(p=%.2f,t=%.2f) cmd(p=%.2f,t=%.2f)",
      idx8, x_norm, y_norm, r_pan, r_tilt, kpan, pan_ref, kp_pan_eff, kp_tilt_eff, pan_out, tilt_out);
  }

private:
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pan_tilt_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::mutex img_size_mutex_;

  int image_width_;
  int image_height_;
  double hfov_deg_;
  double vfov_deg_;
  double hfov_rad_;
  double vfov_rad_;

  std::string control_mode_;
  double pan_sign_;
  double tilt_sign_;
  bool camera_rotated_180_;

  std::string target_class_id_;

  double base_pan_deg_;
  double base_tilt_deg_;

  double kp_pan_;
  double kp_tilt_;
  double k_gain_pan_;
  double k_gain_tilt_;

  bool use_directional_damping_;
  std::vector<double> k_gain_pan_dir8_;
  std::vector<double> k_gain_tilt_dir8_;

  double neg_pan_threshold_deg_;
  double neg_pan_damping_mult_;
  double pos_pan_damping_mult_;

  double edge_damping_pan_;
  double edge_damping_tilt_;

  double pan_min_deg_;
  double pan_max_deg_;
  double tilt_min_deg_;
  double tilt_max_deg_;

  bool use_smoothing_;
  double smoothing_factor_;
  double max_pan_speed_deg_per_sec_;
  double max_tilt_speed_deg_per_sec_;

  double pan_offset_deg_;
  double tilt_offset_deg_;

  double target_distance_m_;
  std::vector<double> dist_points_;
  std::vector<double> pan_offset_points_;
  std::vector<double> tilt_offset_points_;

  bool use_image_topic_size_;
  std::string image_topic_for_size_;

  double prev_pan_cmd_deg_;
  double prev_tilt_cmd_deg_;
  bool prev_cmd_initialized_;
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PanTiltTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
