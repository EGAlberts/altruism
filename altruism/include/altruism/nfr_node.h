#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "altruism_msgs/msg/objects_identified.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
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
 * 
 * 
 * 
 * Note: If in the future a BT should be designed with multiple instances of the same NFR, it would become impractical. Right now the only feasible case it having two NFRs which are never active simultaneously. 
 * In all other cases it would require creating a new (very similar) subclass as the linked blackboard entries would otherwise cause issue.
 * Additionally, the existing Arborist Node services for grabbing NFRs rely on the xml entry for the NFR being unique.. these would need to be updated if the latter were resolved somehow.
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

  //Name for the weight input port
  static constexpr const char* WEIGHT = "weight";

  //Name for the metric input port
  static constexpr const char* METRIC = "metric";

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
      return {InputPort<double>(WEIGHT, "How much influence this NFR should have in the calculation of system utility"), 
      OutputPort<double>(METRIC, "To what extent is this property fulfilled")};
    }

    virtual void calculate_measure() override
    {
      //std::cout << "Here's where I calculate a SafetyNFR measure" << std::endl;
      
      setOutput(METRIC,0.0);

    }



};

class MissionCompleteNFR : public NFRNode
{
  public:
    int counter;
    double detection_threshold; 
    std::string goal_object;
    double times_detected;
    builtin_interfaces::msg::Time last_timestamp;
    double max_detected;
    int detected_in_window;
    int window_length;
    int window_start;
    MissionCompleteNFR(const std::string& name, const NodeConfig& config) : NFRNode(name, config)
    {
      std::cout << "Someone made me (a MissionComplete NFR node) \n\n\n\n\n\n" << std::endl;
      counter = 0;
      window_length = 20;
      detection_threshold = 10; //how many times the object needs to be found in a given position to be confirmed to be there.
      max_detected = 1 * window_length; //What we presume is the max number of objects that'll be detected in a 20 second window.
      //TODO: make this a parameter.
      goal_object = "fire hydrant";
      times_detected = 0; //Also should be a parameter somehow.
      detected_in_window = 0.0;
      window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      

    }

    static PortsList providedPorts()
    {
      return {InputPort<double>(WEIGHT, "How much influence this NFR should have in the calculation of system utility"), 
              InputPort<geometry_msgs::msg::PoseStamped>("rob_position","Robot's current position"),
              InputPort<altruism_msgs::msg::ObjectsIdentified>("objs_identified","The objects detected through the robot's camera"),
              OutputPort<double>(METRIC, "To what extent is this property fulfilled")};
    }

    virtual void calculate_measure() override
    {
      geometry_msgs::msg::PoseStamped some_pose; //probably won't be used after all..
      altruism_msgs::msg::ObjectsIdentified some_objects;
      //std::cout << "Here's where I calculate a MissionCompleteness measure" << std::endl;
      getInput("rob_position", some_pose);
      getInput("objs_identified", some_objects);

      counter += 1;
      if((some_objects.object_detected == true) && (some_objects.stamp != last_timestamp))
      {
        // if(std::find(some_objects.object_names.begin(), some_objects.object_names.end(), goal_object) != std::end(some_objects.object_names)){
        //   times_detected += 1;
        //   last_timestamp = some_objects.stamp;
        // }
        detected_in_window += some_objects.object_names.size();
        last_timestamp = some_objects.stamp;
      }

      //double detection_ratio = times_detected / detection_threshold;
      double detection_ratio = (double)detected_in_window / max_detected;

      auto curr_time_pointer = std::chrono::system_clock::now();
      
      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-window_start;
      if(elapsed_seconds >= window_length)
      {
        setOutput(METRIC,std::min(detection_ratio,1.0));
        window_start = current_time;
        detected_in_window = 0;
      }
      if((counter % 10000) == 0) {
        std::cout << "\n x from within the NFR mission completeness" << some_pose.pose.position.x << "\n" << std::endl;
        std::cout << "\n obj_id bool from within the NFR mission completeness" << some_objects.object_detected << "\n" << std::endl;
        if(some_objects.object_detected == true)
        {
          for (auto i: some_objects.object_names) std::cout << i << ' ';
        }
        std::cout << "\n detection_ratio " << detection_ratio << "\n" << std::endl;
        std::cout << "\n detected_in_window " << detected_in_window << "\n" << std::endl;



        counter = 0;
      }




    }



};

class EnergyNFR : public NFRNode
{
  public:
    int counter;
    double detection_threshold; 
    std::string goal_object;
    double times_detected;
    builtin_interfaces::msg::Time last_timestamp;
    double max_detected;
    int detected_in_window;
    int window_length;
    int window_start;
    EnergyNFR(const std::string& name, const NodeConfig& config) : NFRNode(name, config)
    {
      std::cout << "Someone made me (a Energy NFR node) \n\n\n\n\n\n" << std::endl;
      counter = 0;
      window_length = 20;
      detection_threshold = 10; //how many times the object needs to be found in a given position to be confirmed to be there.
      max_detected = 1 * window_length; //What we presume is the max number of objects that'll be detected in a 20 second window.
      //TODO: make this a parameter.
      goal_object = "fire hydrant";
      times_detected = 0; //Also should be a parameter somehow.
      detected_in_window = 0.0;
      window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      

    }

    static PortsList providedPorts()
    {

      return {InputPort<double>(WEIGHT, "How much influence this NFR should have in the calculation of system utility"), 
              InputPort<float>("in_voltage","voltage"),
              InputPort<float>("in_temperature","temperature"),
              InputPort<float>("in_current","current"),
              InputPort<float>("in_charge","charge"),
              InputPort<float>("in_capacity","capacity"),
              InputPort<float>("in_design_capacity","design_capacity"),
              InputPort<float>("in_percentage","percentage"),
              OutputPort<double>(METRIC, "To what extent is this property fulfilled")};
    }

    virtual void calculate_measure() override
    {
      float voltage;
      float temperature;
      float current;
      float charge;
      float capacity;
      float design_capacity;
      float percentage;
      //std::cout << "Here's where I calculate a MissionCompleteness measure" << std::endl;
      getInput("in_voltage",voltage);
      getInput("in_temperature",temperature);
      getInput("in_current",current);
      getInput("in_charge",charge);
      getInput("in_capacity",capacity);
      getInput("in_design_capacity",design_capacity);
      getInput("in_percentage",percentage);

      //std::cout << "\n x from within the NFR emergy" << voltage << "\n" << std::endl;

      // counter += 1;
      // if((some_objects.object_detected == true) && (some_objects.stamp != last_timestamp))
      // {
      //   // if(std::find(some_objects.object_names.begin(), some_objects.object_names.end(), goal_object) != std::end(some_objects.object_names)){
      //   //   times_detected += 1;
      //   //   last_timestamp = some_objects.stamp;
      //   // }
      //   detected_in_window += some_objects.object_names.size();
      //   last_timestamp = some_objects.stamp;
      // }

      // //double detection_ratio = times_detected / detection_threshold;
      // double detection_ratio = (double)detected_in_window / max_detected;

      // auto curr_time_pointer = std::chrono::system_clock::now();
      
      // int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      // int elapsed_seconds = current_time-window_start;
      // if(elapsed_seconds >= window_length)
      // {
      //   setOutput(MEASURE_NAME,std::min(detection_ratio,1.0));
      //   window_start = current_time;
      //   detected_in_window = 0;
      // }
      // if((counter % 10000) == 0) {
      //   std::cout << "\n x from within the NFR mission completeness" << some_pose.pose.position.x << "\n" << std::endl;
      //   std::cout << "\n obj_id bool from within the NFR mission completeness" << some_objects.object_detected << "\n" << std::endl;
      //   if(some_objects.object_detected == true)
      //   {
      //     for (auto i: some_objects.object_names) std::cout << i << ' ';
      //   }
      //   std::cout << "\n detection_ratio " << detection_ratio << "\n" << std::endl;
      //   std::cout << "\n detected_in_window " << detected_in_window << "\n" << std::endl;



      //   counter = 0;
      // }




    }



};



}   // namespace BT
