// Copyright (c) 2026 Sawyer. MIT license; see repository LICENSE.
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>
#include "gazebo_ros2_control/gazebo_system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ep_gazebo_control
{
// 以受限速度电机追踪位置目标，保留 Gazebo 接触与受力求解。
class PositionServo : public gazebo_ros2_control::GazeboSystemInterface
{
  // 单轴反馈与命令缓存；source 为 -1 表示独立轴，否则索引其跟随源。
  struct Axis
  {
    gazebo::physics::JointPtr joint;
    std::string name;
    double position = 0, velocity = 0, effort = 0, target = 0;
    double gain = 20, max_velocity = 0.5, max_effort = 20;
    int source = -1;
    double multiplier = 1;
  };
  std::vector<Axis> axes_;

public:
  // 绑定 Gazebo 关节，读取伺服参数，并检查 mimic 源先于跟随轴初始化。
  bool initSim(rclcpp::Node::SharedPtr & node, gazebo::physics::ModelPtr model,
    const hardware_interface::HardwareInfo & info, sdf::ElementPtr) override
  {
    nh_ = node;
    axes_.resize(info.joints.size());
    std::map<std::string, int> indices;
    for (size_t i = 0; i < axes_.size(); ++i) indices[info.joints[i].name] = i;
    for (size_t i = 0; i < axes_.size(); ++i) {
      auto & a = axes_[i];
      const auto & spec = info.joints[i];
      a.name = spec.name;
      a.joint = model->GetJoint(a.name);
      if (!a.joint) {
        RCLCPP_ERROR(nh_->get_logger(), "Missing Gazebo joint %s", a.name.c_str());
        return false;
      }
      a.position = a.target = a.joint->Position(0);
      auto parameter = [&](const char * key, double fallback) {
          auto found = spec.parameters.find(key);
          return found == spec.parameters.end() ? fallback : std::stod(found->second);
        };
      a.gain = parameter("servo_gain", 20);
      a.max_velocity = parameter("max_velocity", 0.5);
      a.max_effort = parameter("max_effort", 20);
      auto mimic = spec.parameters.find("mimic");
      if (mimic != spec.parameters.end()) {
        if (!indices.count(mimic->second) || indices.at(mimic->second) >= static_cast<int>(i)) {
          throw std::runtime_error("Mimic sources must precede followers");
        }
        a.source = indices.at(mimic->second);
        a.multiplier = parameter("multiplier", 1);
      }
      a.joint->SetParam("fmax", 0, a.max_effort);
    }
    RCLCPP_INFO(nh_->get_logger(), "EP bounded motor servo initialized (%zu joints)", axes_.size());
    return true;
  }

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override
  {return configure_default(info);}

  // 导出实际位置、速度与力矩缓存，控制器通过这些地址读取反馈。
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> result;
    for (auto & a : axes_) {
      result.emplace_back(a.name, "position", &a.position);
      result.emplace_back(a.name, "velocity", &a.velocity);
      result.emplace_back(a.name, "effort", &a.effort);
    }
    return result;
  }

  // 仅独立轴接收外部位置命令，跟随轴的目标在 write 中计算。
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> result;
    for (auto & a : axes_) {
      if (a.source < 0) result.emplace_back(a.name, "position", &a.target);
    }
    return result;
  }

  hardware_interface::return_type start() override
  {status_ = hardware_interface::status::STARTED; return hardware_interface::return_type::OK;}

  // 停止时将所有轴的速度目标清零。
  hardware_interface::return_type stop() override
  {
    for (auto & a : axes_) a.joint->SetParam("vel", 0, 0.0);
    status_ = hardware_interface::status::STOPPED;
    return hardware_interface::return_type::OK;
  }

  // 每个控制周期从物理引擎读取状态，而非用命令值充当反馈。
  hardware_interface::return_type read() override
  {
    for (auto & a : axes_) {
      a.position = a.joint->Position(0);
      a.velocity = a.joint->GetVelocity(0);
      a.effort = a.joint->GetForce(0);
    }
    return hardware_interface::return_type::OK;
  }

  // 位置误差乘比例增益后限速；非有限目标或反馈使该轴速度目标为零。
  hardware_interface::return_type write() override
  {
    for (auto & a : axes_) {
      if (a.source >= 0) a.target = axes_[a.source].target * a.multiplier;
      double velocity = 0;
      if (std::isfinite(a.target) && std::isfinite(a.position)) {
        velocity = std::max(-a.max_velocity,
          std::min(a.max_velocity, a.gain * (a.target - a.position)));
      }
      // ODE motor constraints supply bounded force. Never teleport links/cube.
      a.joint->SetParam("fmax", 0, a.max_effort);
      a.joint->SetParam("vel", 0, velocity);
    }
    return hardware_interface::return_type::OK;
  }
};
}  // namespace ep_gazebo_control

PLUGINLIB_EXPORT_CLASS(ep_gazebo_control::PositionServo, gazebo_ros2_control::GazeboSystemInterface)
