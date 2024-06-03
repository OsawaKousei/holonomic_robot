#include <ignition/gui/Plugin.hh>
#include <ignition/gui/qt.h>
#include <ignition/transport/Node.hh>

namespace holonomic_sim {

class PcMappingSwitch : public ignition::gui::Plugin {
  Q_OBJECT

public:
  PcMappingSwitch();
  virtual ~PcMappingSwitch();
  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

protected slots:
  void OnSaveMapButton(void);

private:
  void CreateIgnitionIf(void);

private:
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher PcMapping_pub_;
};

} // namespace holonomic_sim
