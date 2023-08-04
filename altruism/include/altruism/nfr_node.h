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

  NFRNode(const std::string& name, const NodeConfig& config) : DecoratorNode(name, config)
  {
    std::cout << "Someone made me (an NFR node) \n\n\n\n\n\n" << std::endl;

    // if (!getInput(WEIGHT, weight_))
    // {
    //   throw RuntimeError("Missing parameter [", WEIGHT, "] in NFRNode");
    // }

  }


  virtual ~NFRNode() override = default;
  static constexpr const char* WEIGHT = "weight";
  static constexpr const char* PROPERTY_NAME = "property_name";



private:
  int weight_;
  std::string property_name_;
  
  NonFunctionalProperty prop_type;
  bool read_parameter_from_ports_;
  static std::map<std::string, NonFunctionalProperty> s_mapNFPs;


  virtual NodeStatus tick() override
  {
    setStatus(NodeStatus::RUNNING);
    const NodeStatus child_status = child_node_->executeTick();
    //std::cout << "I (an NFR node) just ticked my child \n\n\n\n\n\n" << std::endl;


    switch (child_status)
    {
      case NodeStatus::SUCCESS: {
        resetChild();
        return NodeStatus::SUCCESS;
      }

      case NodeStatus::FAILURE: {
        resetChild();
        return NodeStatus::FAILURE;
      }

      case NodeStatus::RUNNING: {
        calculate_measure();
        return NodeStatus::RUNNING;
      }

      case NodeStatus::SKIPPED: {
        return NodeStatus::SKIPPED;
      }
      case NodeStatus::IDLE: {
        throw LogicError("[", name(), "]: A child should not return IDLE");
      }
    }
    return status();

  }

  virtual void calculate_measure()
  {
    getInput(WEIGHT, weight_);
    std::stringstream ss;

    ss << "Weight Port info received: ";
    // for (auto number : feedback->left_time) {
    ss << weight_;
    std::cout << ss.str().c_str() << std::endl;
    std::cout << "Here's where I would calculate a measure... if I had one" << std::endl;
  }

  //void halt() override;
};

class SafetyNFR : public NFRNode
{
  public: 
    SafetyNFR(const std::string& name, const NodeConfig& config) : NFRNode(name, config)
    {
      std::cout << "Someone made me (a Safety NFR node) \n\n\n\n\n\n" << std::endl;
    }

    static PortsList providedPorts()
    {
      return {InputPort<int>(WEIGHT, "How much influence this NFR should have in the calculation of system utility"), 
      OutputPort<float>(MEASURE_NAME, "To what extent is this property fulfilled")};
    }

    virtual void calculate_measure() override
    {
      //std::cout << "Here's where I calculate a SafetyNFR measure" << std::endl;
      
      setOutput(MEASURE_NAME,0.0);

    }
    static constexpr const char* MEASURE_NAME = "safety_metric";



};

class MissionCompleteNFR : public NFRNode
{
  public: 
    MissionCompleteNFR(const std::string& name, const NodeConfig& config) : NFRNode(name, config)
    {
      std::cout << "Someone made me (a MissionComplete NFR node) \n\n\n\n\n\n" << std::endl;
    }

    static PortsList providedPorts()
    {
      return {InputPort<int>(WEIGHT, "How much influence this NFR should have in the calculation of system utility"), 
      OutputPort<float>(MEASURE_NAME, "To what extent is this property fulfilled")};
    }

    virtual void calculate_measure() override
    {
      //std::cout << "Here's where I calculate a MissionCompleteness measure" << std::endl;
      
      setOutput(MEASURE_NAME,0.0);

    }
    static constexpr const char* MEASURE_NAME = "mission_metric";



};


}   // namespace BT
