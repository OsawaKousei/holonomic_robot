#include <ignition/gui/Plugin.hh>
#include <ignition/gui/qt.h>
#include <ignition/transport/Node.hh>

namespace holonomic_sim {

class LocalizationSwitch : public ignition::gui::Plugin {
  Q_OBJECT

public:
  LocalizationSwitch();
  virtual ~LocalizationSwitch();
  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

protected slots:
  void OnLocalizeButton(void);

private:
  void CreateIgnitionIf(void);

private:
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher localize_pub_;
};

} // namespace holonomic_sim
