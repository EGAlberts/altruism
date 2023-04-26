#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "altruism_msgs/action/bandit.hpp"

using namespace BT;




using Bandit = altruism_msgs::action::Bandit;
class BanditAction: public RosActionNode<Bandit>
{
public:
  BanditAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<Bandit>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("rb_name")});
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override 
  {
    //goal->parameters = NULL;
    // // get "radius" from the Input port
    // getInput("rb_name", goal.parameters);
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
    std::stringstream ss;
    ss << "Feedback received: ";
    // for (auto number : feedback->left_time) {
    ss << feedback->chosen_arm;
    // }
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::RUNNING;
  }
};


// static const char* xml_text = R"(
// <root BTCPP_format="4">
//   <BehaviorTree ID="Untitled">
//     <Parallel failure_count="1"
//               success_count="-1">
//       <NFR property="safety"
//            weight="1.0">
//         <Sequence>
//           <Waiting time="10"/>
//           <NFR property="cspeed"
//                weight="1.0">
//             <DriveForward name="Patrol" time="10"/>
//           </NFR>
//         </Sequence>
//       </NFR>
//       <Bandit rb_name="fd"/>
//     </Parallel>
//   </BehaviorTree>
// </root>
// )";


static const char* xml_text = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Untitled">
    <Bandit rb_name="fd"/>
  </BehaviorTree>
</root>
)";

int main(int argc, char * argv[])
{
  std::cout << "Now I'm here and I wish that I wasn't";
  

  rclcpp::init(argc, argv);
  
  auto nh = std::make_shared<rclcpp::Node>("bt_bandit_client");

  BehaviorTreeFactory factory;


  RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "bandit";


  factory.registerNodeType<BanditAction>("Bandit", params);


  auto tree = factory.createTreeFromText(xml_text);

  tree.tickWhileRunning();

  return 0;
}
