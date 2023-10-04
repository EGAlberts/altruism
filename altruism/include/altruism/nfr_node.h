#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "altruism_msgs/msg/objects_identified.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "altruism/system_attribute_value.hpp"

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

    _average_metric = 0.0;
    _times_calculated = 0;
    // if (!getInput(WEIGHT, weight_))
    // {
    //   throw RuntimeError("Missing parameter [", WEIGHT, "] in NFRNode");
    // }

  }

  static PortsList providedPorts()
  {
      return {InputPort<double>(WEIGHT, "How much influence this NFR should have in the calculation of system utility"), 
              OutputPort<double>(METRIC, "To what extent is this property fulfilled"),
              OutputPort<double>(MEAN_METRIC, "To what extent is this property fulfilled on average")};
  }


  virtual ~NFRNode() override = default;

  //Name for the weight input port
  static constexpr const char* WEIGHT = "weight";

  //Name for the metric input port
  static constexpr const char* METRIC = "metric";
  static constexpr const char* MEAN_METRIC = "mean_metric";


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

  protected:
    int _times_calculated;
    double _average_metric;
    double _metric;

    void metric_mean()
    {
      double new_average = _average_metric + ((_metric - _average_metric) / (double)_times_calculated);
      _average_metric = new_average;
    }

    void output_metric()
    {
      setOutput(METRIC,std::min(_metric,1.0));
      _times_calculated++; 
    }


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
      PortsList base_ports = NFRNode::providedPorts();

      PortsList child_ports = {};

      child_ports.merge(base_ports);

      return child_ports;
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
    MissionCompleteNFR(const std::string& name, const NodeConfig& config) : NFRNode(name, config)
    {
      std::cout << "Someone made me (a MissionComplete NFR node) \n\n\n\n\n\n" << std::endl;
      _counter = 0;

      _detected_in_window = 0.0;
      _window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

      _last_timestamp = builtin_interfaces::msg::Time();
  
    }

    void initialize(double max_objs_ps, int window_length)
    {
        _max_object_ps = max_objs_ps;
        _window_length = window_length;

        _max_detected = _max_object_ps * double(_window_length); //What we presume is the max number of objects that'll be detected in a 20 second window.
    }


    static PortsList providedPorts()
    {
      PortsList base_ports = NFRNode::providedPorts();

      PortsList child_ports =  { 
              InputPort<geometry_msgs::msg::PoseStamped>("rob_position","Robot's current position"),
              InputPort<altruism_msgs::msg::ObjectsIdentified>("objs_identified","The objects detected through the robot's camera")};
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {
      altruism_msgs::msg::ObjectsIdentified objects_msg;

      getInput("objs_identified", objects_msg);

      _counter += 1;
      if((objects_msg.object_detected == true) && (objects_msg.stamp != _last_timestamp))
      {
        _detected_in_window += objects_msg.object_names.size();
        _last_timestamp = objects_msg.stamp;
      }

      _metric = (double)_detected_in_window / _max_detected;

      auto curr_time_pointer = std::chrono::system_clock::now();
      
      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-_window_start;
      if(elapsed_seconds >= _window_length)
      {
        output_metric();
        metric_mean();
        std::cout << "the mean metric " << _average_metric << std::endl;
        setOutput(MEAN_METRIC,_average_metric);
        _window_start = current_time;
        _detected_in_window = 0;
      }

      if((_counter % 10000) == 0) {
        std::cout << "\n obj_id bool from within the NFR mission completeness" << objects_msg.object_detected << "\n" << std::endl;
        if(objects_msg.object_detected == true)
        {
          for (auto i: objects_msg.object_names) std::cout << i << ' ';
        }
        std::cout << "\n detection_ratio " << _metric << "\n" << std::endl;
        std::cout << "\n detected_in_window " << _detected_in_window << "\n" << std::endl;



        _counter = 0;
      }

    }

    private:
      int _detected_in_window;
      builtin_interfaces::msg::Time _last_timestamp;
      double _max_detected;
      double _max_object_ps;
      int _window_length;
      int _window_start;
      int _counter;

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
    int window_start;
    EnergyNFR(const std::string& name, const NodeConfig& config) : NFRNode(name, config)
    {
      std::cout << "Someone made me (a Energy NFR node) \n\n\n\n\n\n" << std::endl;
      counter = 0;

      _window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      _last_odom = _window_start;
      _obj_last_timestamp = builtin_interfaces::msg::Time();
      _odom_last_timestamp_sec = 0;
      
      _motion_consumption = 0.0;
      _idle_consumption = 0.0;
      _laser_consumption = 0.0;

      _pictures_taken_in_window = 0;


      
    }

    void initialize(float max_pics_ps, int window_length)
    {
        _max_pics_ps = max_pics_ps;
        _window_length = window_length;

        _max_motion_consumption =  ((6.25 * pow(WAFFLE_MAX_LIN_VEL, 2.0) + 9.79 * WAFFLE_MAX_LIN_VEL +3.66)) * (_window_length+1);
        _max_picture_consumption = (max_pics_ps * (_window_length+1)) * detection_average_power;
        _max_idle_consumption = _idle_power * (_window_length + 1);
        _max_laser_consumption = _laser_power * (_window_length + 1);

        _max_consumption = _max_motion_consumption + _max_picture_consumption + _max_idle_consumption +  _max_laser_consumption;
    }


    static PortsList providedPorts()
    {
      PortsList base_ports = NFRNode::providedPorts();

      PortsList child_ports = { 
              InputPort<altruism::SystemAttributeValue>(IN_ODOM,"odometry message wrapped in a systemattributevalue instance"),
              InputPort<int32_t>(IN_PIC_RATE,"picture rate of the ID child action"),
              InputPort<altruism_msgs::msg::ObjectsIdentified>("objs_identified","The objects detected through the robot's camera")
              };

      child_ports.merge(base_ports);

      return child_ports;
    }

    static float calculate_power_motion(float speed) {
      return 6.25 * pow(speed, 2) + 9.79 * speed + 3.66;
    }
	
    virtual void calculate_measure() override
    {
      auto res = getInput(IN_ODOM,_odom_attribute); 
      getInput(IN_PIC_RATE,_pic_rate);

      getInput("objs_identified", _objects_msg);

      if(res)
      {
        _odom_msg = _odom_attribute.get<altruism::SystemAttributeType::ATTRIBUTE_ODOM>();
        _linear_speed = hypot(fabs(_odom_msg.twist.twist.linear.x), fabs(_odom_msg.twist.twist.linear.y));
      }

      if(_objects_msg.stamp != _obj_last_timestamp)
      {
        std::cout << "\n\n\n this done happened \n\n\n" << std::endl;
        _pictures_taken_in_window += _pic_rate;
        _obj_last_timestamp = _objects_msg.stamp;
      }


      if( (_odom_msg.header.stamp.sec-_odom_last_timestamp_sec) >= 1 ) //one second has passed since last odom
      {
        
        _motion_consumption+= calculate_power_motion(_linear_speed);
        _laser_consumption+= _laser_power;
        _idle_consumption+=  _idle_power;

        _odom_last_timestamp_sec = _odom_msg.header.stamp.sec;
        
      }

      //PR_Motion(v = 6.25⋅v2+9.79⋅v+3.66 
      


      auto curr_time_pointer = std::chrono::system_clock::now();

      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-_window_start;
      if(elapsed_seconds >= _window_length)
      {
        float picture_consumption = _pictures_taken_in_window * detection_average_power;

        std::cout << "pic in window " << _pictures_taken_in_window << " det avg " << detection_average_power << std::endl;
        std::cout << "components to power consumption  " << _motion_consumption << " " <<  picture_consumption << " " << _idle_consumption << " " << _laser_consumption << std::endl;

        float total_consumption = _motion_consumption + picture_consumption + _idle_consumption +  _laser_consumption;


        _metric = 1 - (total_consumption/_max_consumption); //1 - because power consumption is a bad thing for the QA
        std::cout << "total" << total_consumption  << " divided by " << "max" << _max_consumption  << std::endl;

        output_metric();
        std::cout << "the energy metric " << _metric << std::endl;

        metric_mean();

        std::cout << "the mean energy metric " << _average_metric << std::endl;

        setOutput(MEAN_METRIC,_average_metric);

        _window_start = current_time;
        _motion_consumption = 0.0;
        _laser_consumption = 0.0;
        _idle_consumption = 0.0;

        _pictures_taken_in_window = 0;
      }

    }
  private:
      float _max_pics_ps;
      altruism::SystemAttributeValue _odom_attribute;
      int32_t _pic_rate;
      int _pictures_taken_in_window;
      float _motion_consumption;
      float _max_consumption;
      float _max_motion_consumption;
      float _max_picture_consumption;
      float _max_picture_taken;
      float _idle_consumption;
      float _max_idle_consumption;
      float _laser_consumption;
      float _max_laser_consumption;
      int _window_length;
      int _window_start;
      int _last_odom;
      float _linear_speed;

      nav_msgs::msg::Odometry _odom_msg;
      altruism_msgs::msg::ObjectsIdentified _objects_msg;
      int _odom_last_timestamp_sec;
      builtin_interfaces::msg::Time _obj_last_timestamp;


      const float _idle_power = 1.14; //as caused by the detection software running. Idle consumption of the robot as a whole is considered a constant factor.
      const float detection_average_power = 10.919800; //watts
      const float _laser_power = 2.34; //watts
      const float WAFFLE_MAX_LIN_VEL = 0.26;


      static constexpr const char* IN_PIC_RATE = "in_picture_rate";
      static constexpr const char* IN_ODOM = "in_odom";





};



}   // namespace BT
