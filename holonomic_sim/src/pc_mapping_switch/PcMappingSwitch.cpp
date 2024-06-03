#include "PcMappingSwitch.hpp"

#include <ignition/msgs/float.pb.h>
#include <ignition/plugin/Register.hh>

namespace holonomic_sim {

PcMappingSwitch::PcMappingSwitch() : Plugin() { CreateIgnitionIf(); }

PcMappingSwitch::~PcMappingSwitch() {}

void PcMappingSwitch::LoadConfig(const tinyxml2::XMLElement *_pluginElem) {
  if (!_pluginElem) {
    return;
  }
}

void PcMappingSwitch::OnSaveMapButton(void) {
  ignition::msgs::StringMsg string_msg;
  string_msg.set_data("save_map");
  PcMapping_pub_.Publish(string_msg);
}

void PcMappingSwitch::CreateIgnitionIf(void) {
  this->PcMapping_pub_ =
      this->node_.Advertise<ignition::msgs::StringMsg>("pc_mapping_cmd");
}

} // namespace holonomic_sim

// Register this plugin
IGNITION_ADD_PLUGIN(holonomic_sim::PcMappingSwitch, ignition::gui::Plugin)
