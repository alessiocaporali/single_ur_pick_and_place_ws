#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class JointStateSplitter : public rclcpp::Node
{
public:
  JointStateSplitter()
  : rclcpp::Node("joint_state_splitter"),
    rate_(500.0)
  {
    // Declare parameters
    source_topic_ = this->declare_parameter<std::string>("source_topic", "/joint_states");
    ns_      = this->declare_parameter<std::string>("sim_namespace", "ur_sim");
    joints_  = this->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});

    // Build output topics (like f"/{ns}/joint_states".replace("//","/"))
    sim_topic_  = "/" + ns_ + "/joint_states";
    normalize_topic(sim_topic_);

    // Subscription
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      source_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&JointStateSplitter::joint_state_cb, this, _1));

    // Publishers
    sim_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(sim_topic_, 10);

    // Timer at 500 Hz
    auto period = std::chrono::duration<double>(1.0 / rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&JointStateSplitter::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribing to '%s', publishing to '%s'",
      source_topic_.c_str(), sim_topic_.c_str());
  }


private:
  void timer_callback()
  {
    sim_pub_->publish(sim_msg_);
  }

  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Reset messages on each incoming joint_state
    sim_msg_.header = msg->header;
    sim_msg_.name.clear();
    sim_msg_.position.clear();
    sim_msg_.velocity.clear();
    sim_msg_.effort.clear();

    const auto & names = msg->name;
    const auto & pos   = msg->position;
    const auto & vel   = msg->velocity;
    const auto & eff   = msg->effort;

    for (size_t i = 0; i < names.size(); ++i) {
      const std::string & name = names[i];

      double p = (i < pos.size()) ? pos[i] : 0.0;
      double v = (i < vel.size()) ? vel[i] : 0.0;
      double e = (i < eff.size()) ? eff[i] : 0.0;

      // Prefer explicit joint lists if provided
      if (!joints_.empty() && std::find(joints_.begin(), joints_.end(), name) != joints_.end())
      {
        append_joint(sim_msg_, name, p, v, e);
      }
      else {
        // If no explicit joints are defined, include all joints
        append_joint(sim_msg_, name, p, v, e);
      }

      
      }
    
  }

  static void append_joint(sensor_msgs::msg::JointState & msg,
                           const std::string & name,
                           double p, double v, double e)  
  {
    msg.name.push_back(name);
    msg.position.push_back(p);
    msg.velocity.push_back(v);
    msg.effort.push_back(e);
  }

  static bool starts_with(const std::string & s, const std::string & prefix)
  {
    return s.rfind(prefix, 0) == 0;  // prefix at position 0
  }

  static void normalize_topic(std::string & topic)
  {
    // crude "//" → "/"
    std::string out;
    out.reserve(topic.size());
    bool last_was_slash = false;
    for (char c : topic) {
      if (c == '/') {
        if (!last_was_slash) {
          out.push_back(c);
          last_was_slash = true;
        }
      } else {
        out.push_back(c);
        last_was_slash = false;
      }
    }
    topic = out;
  }

  double rate_;

  std::string source_topic_;
  std::string ns_;
  std::vector<std::string> joints_;

  std::string sim_topic_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState sim_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateSplitter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
