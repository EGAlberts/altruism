#pragma once

#include "behaviortree_cpp/decorator_node.h"

namespace BT
{

enum class NonFunctionalProperty
{
  COMPLETION_SPEED,
  SAFETY,
  ENERGY_EFFICIENCY,
};
/**
 * @brief The NFRNode is used to ...
 *
 *
 * 
 *
 * 
 * 
 *
 * Example:
 *
 * <Repeat num_cycles="3">
 *   <ClapYourHandsOnce/>
 * </Repeat>
 */
class NFRNode : public DecoratorNode
{
public:

  NFRNode(const std::string& name, const NodeConfig& config);


  virtual ~NFRNode() override = default;

  static PortsList providedPorts()
  {
    return {InputPort<int>(weight, "How much influence this NFR should have in the calculation of system utility"), InputPort<std::string>(property_name, "Which property does this NFR concern e.g. safety")};
  }

private:
  int weight_;
  std::string property_name_;
  
  NonFunctionalProperty prop_type;
  bool read_parameter_from_ports_;
  static std::map<std::string, NonFunctionalProperty> s_mapNFPs;
  static constexpr const char* WEIGHT = "weight";
  static constexpr const char* PROPERTY_NAME = "property_name";


  virtual NodeStatus tick() override;

  void halt() override;
};

}   // namespace BT
