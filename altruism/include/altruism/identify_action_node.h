#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "altruism_msgs/action/identify.hpp"
#include "altruism_msgs/msg/objects_identified.hpp"



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
    return providedBasicPorts({OutputPort<altruism_msgs::msg::ObjectsIdentified>("objs_identified")});
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override 
  {
    std::string some_text;
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
    // std::stringstream ss;
    // ss << "Result received: ";
    // for (auto number : wr.result) {
    //   ss << number << " ";
    // }
    // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
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
