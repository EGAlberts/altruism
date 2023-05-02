#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "altruism_msgs/action/bandit.hpp"
#include "altruism_msgs/action/behavior_tree.hpp"

#include "altruism_msgs/srv/set_blackboard.hpp"

#include "altruism/bandit_action_node.h"



using namespace BT;
using std::placeholders::_1;
using std::placeholders::_2;

static const char* half_xml_text = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Untitled">
    <Parallel failure_count="1"
              success_count="-1">
      <NFR property="safety"
           weight="1.0">
        <Sequence>
          <NFR property="cspeed"
               weight="1.0">
            <SLAM_fd/>
          </NFR>
          <NFR property="cspeed, energy"
               weight="1.0, 5.0">
            <SearchAndID_fd/>
          </NFR>
          <Pass_fd/>
        </Sequence>
      </NFR>
      <Bandit rb_name="dl"/>
    </Parallel>
  </BehaviorTree>
</root>
)";


static const char* bandit_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Untitled">
    <Sequence>
      <Script code=" the_answer:='SomeText' " />
      <Bandit rb_name="{the_answer}"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

static const char* xml_tree = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Untitled">
    <Sequence>
      <Bandit rb_name="{the_answer}"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

static const char* bandit_xml2 = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Untitled">
    <Sequence>
      <Bandit rb_name="{the_answer}"/>
    </Sequence>
  </BehaviorTree>
</root>
)";



class Arborist : public rclcpp::Node
{
public:
  using SetBlackboard = altruism_msgs::srv::SetBlackboard;
  using BTAction = altruism_msgs::action::BehaviorTree;
  using GoalHandleBTAction = rclcpp_action::ServerGoalHandle<BTAction>;

  BT::Tree tree;
  BehaviorTreeFactory factory;

  Arborist(std::string name = "arborist_node") : Node(name)
  {
      _set_blackboard = this->create_service<SetBlackboard>("set_blackboard", std::bind(&Arborist::handle_set_bb, this, _1, _2));
      //_start_tree = this->create_service<SetBlackboard>("set_blackboard", std::bind(&Arborist::handle_set_bb, this, _1, _2));
      _start_tree = rclcpp_action::create_server<BTAction>(
      this,
      "start_tree",
      std::bind(&Arborist::handle_tree_goal, this, _1, _2),
      std::bind(&Arborist::handle_tree_cancel, this, _1),
      std::bind(&Arborist::handle_tree_accepted, this, _1));

      registerCustomNode(factory, "bt_bandit_client", "bandit", "Bandit");
      tree = factory.createTreeFromText(xml_tree);
  }

  void registerCustomNode(BehaviorTreeFactory& factory, std::string client_name, std::string action_name, std::string name_in_xml)
  {
    auto nh = std::make_shared<rclcpp::Node>(client_name);

    RosNodeParams params;
    params.nh = nh;
    params.default_port_value = action_name;

    factory.registerNodeType<BanditAction>(name_in_xml, params);

  }

private:
  void handle_set_bb(const std::shared_ptr<SetBlackboard::Request> request,
        std::shared_ptr<SetBlackboard::Response> response)
  {
    std::cout<<"This is service 1"<<std::endl;

    std::string script = " the_answer:='SomeOtherText' ";
    
    std::string _script;
    ScriptFunction _executor;

    auto executor = ParseScript(script);
    if (!executor)
    {
      throw RuntimeError(executor.error());
    }
    else
    {
      _executor = executor.value();
      _script = script;
    }

    if (_executor)
    {
      Ast::Environment env = {tree.rootBlackboard(), nullptr};
      _executor(env);
    }
  }

  void handle_start_tree(const std::shared_ptr<GoalHandleBTAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal: Starting to tick tree");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<BTAction::Feedback>();
    feedback->node_status = "placeholder";

    auto result = std::make_shared<BTAction::Result>();

    NodeStatus result_of_tick;


    do {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->is_success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled: didn't finish BT");
        return;
      }
      result_of_tick = tree.tickOnce();
      feedback->node_status = toStr(result_of_tick);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      // RCLCPP_INFO(this->get_logger(), "Publish feedback");

      // Check if goal is done
      if(result_of_tick != NodeStatus::RUNNING) {
        if (rclcpp::ok()) {

          switch(result_of_tick) {
            case NodeStatus::SUCCESS:
              result->is_success = true;
              break;
            case NodeStatus::FAILURE:
              result->is_success = false;
              break;
          }
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal finished: Done ticking the tree");
          break;
        }

      }
    }
    while(result_of_tick == NodeStatus::RUNNING);
  }


  rclcpp_action::GoalResponse handle_tree_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const BTAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_tree_cancel(
    const std::shared_ptr<GoalHandleBTAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_tree_accepted(const std::shared_ptr<GoalHandleBTAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Handle accepted callback");

    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Arborist::handle_start_tree, this, _1), goal_handle}.detach();
  }

  rclcpp::Service<SetBlackboard>::SharedPtr _set_blackboard;
  rclcpp_action::Server<BTAction>::SharedPtr _start_tree;

  
    
    
};


int main(int argc, char * argv[])
{
  std::cout << "Now I'm here and I wish that I wasn't";
  

  rclcpp::init(argc, argv);

  auto node = std::make_shared<Arborist>();
  rclcpp::spin(node);
  rclcpp::shutdown();




  //return 0;
}
