/**
 * @file holonomic_plugin.hpp
 * @brief Holonomic plugin for holonomic simulation
 * @author kousei
 * @date 2024-05-29
*/
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>

namespace holonomic_sim {
class HolonomicPlugin : public ignition::gazebo::System,
                        public ignition::gazebo::ISystemConfigure,
                        public ignition::gazebo::ISystemPreUpdate,
                        public ignition::gazebo::ISystemUpdate,
                        public ignition::gazebo::ISystemPostUpdate {
public:
  HolonomicPlugin();
  ~HolonomicPlugin() override;
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;

  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;
  void Update(const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override;
  void
  PostUpdate(const ignition::gazebo::UpdateInfo &_info,
             const ignition::gazebo::EntityComponentManager &_ecm) override;

private:
  void CreateIgnitionIf(void);
  void OnCmdVelMessage(const ignition::msgs::Twist &msg);

  ignition::gazebo::Model model_;
  ignition::transport::Node node_;
  std::string wheel_joint_name_{""};
  float wheel_vel_{0.0f};
  std::string top_base_joint_name_{""};
  std::string bottom_base_joint_name_{""};
  float base_pos_{0.0f};
  float base_vel_{0.0f};
  const float wheel_radius_{0.2f}; // m
  float wheel_rot_vel_{0.0f};
};
} // namespace holonomic_sim