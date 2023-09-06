#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "altruism_msgs/action/identify.hpp"
#include "altruism_msgs/msg/objects_identified.hpp"
#include "nav_msgs/srv/get_map.hpp"


using namespace BT;


using ID = altruism_msgs::action::Identify;

class IdentifyAction: public RosActionNode<ID>
{
public:
  IdentifyAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<ID>(name, conf, params)
  {
    std::cout << "Someone made me (an ID Action Node) \n\n\n\n\n\n" << std::endl;


  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({OutputPort<altruism_msgs::msg::ObjectsIdentified>("objs_identified"),
                               OutputPort<float>("out_time_elapsed"),
                               OutputPort<int32_t>("out_picture_rate") });
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override 
  {

    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client = node_->create_client<nav_msgs::srv::GetMap>("/slam_toolbox/dynamic_map");
    std::string some_text;

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
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
    setOutput("out_picture_rate", wr.result->picture_rate); 


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
    //of note is that miraculously the BT CPP knows to send this to the current_position field of the blackboard, which is picked up in an NFR, because it is specified in the BT xml
    
    sstwo << "Port info received: ";
    // for (auto number : feedback->left_time) {
    sstwo << some_text;
    // }
    RCLCPP_INFO(node_->get_logger(), sstwo.str().c_str());


    return NodeStatus::RUNNING;
  }
};
