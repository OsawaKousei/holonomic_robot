/**
 * @file holonomic_plugin.cpp
 * @brief Holonomic plugin for holonomic simulation
 * @author kousei
 * @date 2024-05-29
 */
#include "holonomic_plugin.hpp"
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/msgs/twist.pb.h>

namespace holonomic_sim
{

  HolonomicPlugin::HolonomicPlugin() { CreateIgnitionIf(); }

  HolonomicPlugin::~HolonomicPlugin() {}

  void HolonomicPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  ignition::gazebo::EntityComponentManager &_ecm,
                                  ignition::gazebo::EventManager &_eventMgr)
  {
    (void)_ecm;
    (void)_eventMgr;
    model_ = ignition::gazebo::Model(_entity);

    auto wheel_ptr = const_cast<sdf::Element *>(_sdf.get());
    sdf::ElementPtr wheel_joint_elem = wheel_ptr->GetElement("wheel_joint");
    if (wheel_joint_elem)
    {
      wheel_joint_name_ = wheel_joint_elem->Get<std::string>();
    }
    else
    {
      ignerr << "sdf wheel_joint not found" << std::endl;
    }

    auto top_base_ptr = const_cast<sdf::Element *>(_sdf.get());
    sdf::ElementPtr top_base_joint_elem =
        top_base_ptr->GetElement("top_base_joint");
    if (top_base_joint_elem)
    {
      top_base_joint_name_ = top_base_joint_elem->Get<std::string>();
    }
    else
    {
      ignerr << "sdf base_joint not found" << std::endl;
    }

    auto bottom_base_ptr = const_cast<sdf::Element *>(_sdf.get());
    sdf::ElementPtr bottom_base_joint_elem =
        bottom_base_ptr->GetElement("bottom_base_joint");
    if (bottom_base_joint_elem)
    {
      bottom_base_joint_name_ = bottom_base_joint_elem->Get<std::string>();
    }
    else
    {
      ignerr << "sdf base_joint not found" << std::endl;
    }
  }

  void HolonomicPlugin::PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm)
  {
    (void)_info;
    (void)_ecm;
    ignition::gazebo::Entity wheel_joint =
        model_.JointByName(_ecm, wheel_joint_name_);
    if (wheel_joint == ignition::gazebo::kNullEntity)
    {
      ignerr << wheel_joint_name_ << " not found" << std::endl;
      return;
    }

    ignition::gazebo::Entity top_base_joint =
        model_.JointByName(_ecm, top_base_joint_name_);
    if (top_base_joint == ignition::gazebo::kNullEntity)
    {
      ignerr << top_base_joint_name_ << " not found" << std::endl;
      return;
    }

    ignition::gazebo::Entity bottom_base_joint =
        model_.JointByName(_ecm, bottom_base_joint_name_);
    if (bottom_base_joint == ignition::gazebo::kNullEntity)
    {
      ignerr << bottom_base_joint << " not found" << std::endl;
      return;
    }

    auto wheel_rot_vel =
        _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(
            wheel_joint);
    if (wheel_rot_vel != nullptr)
    {
      *wheel_rot_vel =
          ignition::gazebo::components::JointVelocityCmd({wheel_rot_vel_});
    }
    else
    {
      _ecm.CreateComponent(
          wheel_joint,
          ignition::gazebo::components::JointVelocityCmd({wheel_rot_vel_}));
    }

    auto base_pos =
        _ecm.Component<ignition::gazebo::components::JointPositionReset>(
            top_base_joint);
    if (base_pos != nullptr)
    {
      *base_pos = ignition::gazebo::components::JointPositionReset({base_pos_});
    }
    else
    {
      _ecm.CreateComponent(
          bottom_base_joint,
          ignition::gazebo::components::JointPositionReset({base_pos_}));
    }
    _ecm.CreateComponent(top_base_joint,
                         ignition::gazebo::components::JointVelocityCmd({0.0f}));

    auto base_vel =
        _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(
            bottom_base_joint);
    if (base_vel != nullptr)
    {
      *base_vel = ignition::gazebo::components::JointVelocityCmd({base_vel_});
    }
    else
    {
      _ecm.CreateComponent(
          bottom_base_joint,
          ignition::gazebo::components::JointVelocityCmd({base_vel_}));
    }
  }

  void HolonomicPlugin::Update(const ignition::gazebo::UpdateInfo &_info,
                               ignition::gazebo::EntityComponentManager &_ecm)
  {
    (void)_info;
    (void)_ecm;
  }

  void HolonomicPlugin::PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm)
  {
    (void)_info;
    (void)_ecm;
  }

  void HolonomicPlugin::CreateIgnitionIf(void)
  {
    this->node_.Subscribe("cmd_vel", &HolonomicPlugin::OnCmdVelMessage, this);
  }

  void HolonomicPlugin::OnCmdVelMessage(const ignition::msgs::Twist &msg)
  {
    float x = msg.linear().x();
    float y = msg.linear().y();
    wheel_vel_ = sqrt(x * x + y * y);
    wheel_rot_vel_ = wheel_vel_ / wheel_radius_;
    base_pos_ = atan2(y, x);
    base_vel_ = -1 * msg.angular().z();
  }

} // namespace holonomic_sim

#include <ignition/plugin/Register.hh>
IGNITION_ADD_PLUGIN(holonomic_sim::HolonomicPlugin, ignition::gazebo::System,
                    holonomic_sim::HolonomicPlugin::ISystemConfigure,
                    holonomic_sim::HolonomicPlugin::ISystemPreUpdate,
                    holonomic_sim::HolonomicPlugin::ISystemUpdate,
                    holonomic_sim::HolonomicPlugin::ISystemPostUpdate)
