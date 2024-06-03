#include "LocalizationSwitch.hpp"

#include <ignition/msgs/float.pb.h>
#include <ignition/plugin/Register.hh>

namespace holonomic_sim {

LocalizationSwitch::LocalizationSwitch() : Plugin() { CreateIgnitionIf(); }

LocalizationSwitch::~LocalizationSwitch() {}

void LocalizationSwitch::LoadConfig(const tinyxml2::XMLElement *_pluginElem) {
  if (!_pluginElem) {
    return;
  }
}

void LocalizationSwitch::OnLocalizeButton(void) {
  ignition::msgs::StringMsg string_msg;
  string_msg.set_data("localize");
  localize_pub_.Publish(string_msg);
}

void LocalizationSwitch::CreateIgnitionIf(void) {
  this->localize_pub_ =
      this->node_.Advertise<ignition::msgs::StringMsg>("if_localize");
}

} // namespace holonomic_sim

// Register this plugin
IGNITION_ADD_PLUGIN(holonomic_sim::LocalizationSwitch, ignition::gui::Plugin)
