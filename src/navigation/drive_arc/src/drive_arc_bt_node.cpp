#include "drive_arc/drive_arc_bt_node.hpp"
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<drive_arc::DriveArcAction>(
    "DriveArc",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<drive_arc::DriveArcAction>(name, "drive_arc", config);
    });
}
