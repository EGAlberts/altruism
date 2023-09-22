#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "altruism_msgs/action/identify.hpp"
#include "altruism_msgs/msg/variable_parameter.hpp"

#include "altruism_msgs/msg/objects_identified.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "altruism/variable_action_node.h"

using namespace BT;


using ID = altruism_msgs::action::Identify;
using VariableParam = altruism_msgs::msg::VariableParameter;


class IdentifyAction: public VariableActionNode<ID>
{
public:
  static constexpr const char* DET_THRESH = "out_det_threshold";
  static constexpr const char* PC_RATE = "out_picture_rate";

  
  const std::string PICTURE_RT_PARAM = "pic_rate";

  IdentifyAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : VariableActionNode<ID>(name, conf, params)
  {
    std::cout << "Someone made me (an ID Action Node) \n\n\n\n\n\n" << std::endl;
    
    _var_params = VariableParameters();

    VariableParam picture_rate_param = VariableParam();
    VariableParam pret_param = VariableParam();


    picture_rate_param.name = PICTURE_RT_PARAM;
    int pc_rate_values[4] = {1,3,5,7};

    for (int val : pc_rate_values) {
      ParamValue possible_value = ParamValue();
      possible_value.type = 2; //integer type
      possible_value.integer_value = val;
      picture_rate_param.possible_values.push_back(possible_value);
    }

    _var_params.variable_parameters.push_back(picture_rate_param); //vector of VariableParameter

    // pret_param.name = "pretend_var";
    // int pret_values[2] = {10,20};

    // for (int val : pret_values) {
    //   ParamValue possible_value = ParamValue();
    //   possible_value.type = 2; //integer type
    //   possible_value.integer_value = val;
    //   pret_param.possible_values.push_back(possible_value);
    // }

    // _var_params.variable_parameters.push_back(pret_param); //vector of VariableParameter
    
    setOutput(VARIABLE_PARAMS, _var_params);

  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    PortsList base_ports = VariableActionNode::providedPorts();

    PortsList child_ports = { 
              OutputPort<altruism_msgs::msg::ObjectsIdentified>("objs_identified"),
              OutputPort<float>("out_time_elapsed"),
              OutputPort<int32_t>(PC_RATE),
              OutputPort<int32_t>(DET_THRESH) 

            };

    child_ports.merge(base_ports);

    return child_ports;
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(VariableActionNode::Goal& goal) override 
  {

    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr slam_get_mapclient = node_->create_client<nav_msgs::srv::GetMap>("/slam_toolbox/dynamic_map");
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr noslam_get_mapclient = node_->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client;
    std::string some_text;

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    while(true)
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }

      if(!slam_get_mapclient->wait_for_service(std::chrono::seconds(1)))
      {
        RCLCPP_INFO(node_->get_logger(), "SLAM Get map service not available, waiting again...");
      }
      else{
        client = slam_get_mapclient;
        break;
      }

      if(!noslam_get_mapclient->wait_for_service(std::chrono::seconds(1)))
      {
        RCLCPP_INFO(node_->get_logger(), "map server Get map service not available, waiting again...");
      }
      else{
        client = noslam_get_mapclient;
        break;
      }
    }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    goal.map = result.get()->map;
    RCLCPP_INFO(node_->get_logger(), "Success getting map");

  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service get map");
    return false;
  }

    //goal->parameters = NULL;
    // // get "radius" from the Input port
    //getInput("rb_name", some_text);
    std::stringstream ss;
    
    

    ss << "Port info received: ";
    // for (auto number : feedback->left_time) {
    ss << some_text;
    // }
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    // return true, if we were able to set the goal correctly.
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    std::stringstream ss;
    ss << "ID Result received: " << wr.result->time_elapsed << " pic rate" << wr.result->picture_rate;
    setOutput("out_time_elapsed", wr.result->time_elapsed); 
    setOutput(PC_RATE, wr.result->picture_rate); 
    setOutput(DET_THRESH, wr.result->detection_threshold); 



    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    std::string some_text;
    std::stringstream ss;
    ss << "ID Feedback received obj detected: ";
    // for (auto number : feedback->left_time) {
    ss << feedback->obj_idd.object_detected;
    // }
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    //getInput("rb_name", some_text);
    std::stringstream sstwo;
    
    setOutput("objs_identified", feedback->obj_idd); 
    setOutput(PC_RATE, feedback->picture_rate); 

    //of note is that miraculously the BT CPP knows to send this to the current_position field of the blackboard, which is picked up in an NFR, because it is specified in the BT xml
    
    sstwo << "Port info received: ";
    // for (auto number : feedback->left_time) {
    sstwo << some_text;
    // }
    RCLCPP_INFO(node_->get_logger(), sstwo.str().c_str());


    return NodeStatus::RUNNING;
  }
};
