#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "altruism_msgs/action/bandit.hpp"
#include "altruism_msgs/action/behavior_tree.hpp"

#include "altruism_msgs/srv/set_blackboard.hpp"
#include "altruism_msgs/srv/get_blackboard.hpp"
#include "altruism_msgs/srv/get_nfr.hpp"
#include "altruism_msgs/srv/set_weights.hpp"
#include "altruism_msgs/msg/nfr.hpp"




#include "altruism/bandit_action_node.h"
#include "altruism/slam_action_node.h"
#include "altruism/identify_action_node.h"


#include "altruism/nfr_node.h"

#include <fstream>


using namespace BT;
using std::placeholders::_1;
using std::placeholders::_2;

// static const char* half_xml_text = R"(
// <root BTCPP_format="4">
//   <BehaviorTree ID="Untitled">
//     <Parallel failure_count="1"
//               success_count="-1">
  
//     </Parallel>
//   </BehaviorTree>
// </root>
// )";


// static const char* bandit_xml = R"(
// <root BTCPP_format="4">
//   <BehaviorTree ID="Untitled">
//     <Sequence>
//       <Script code=" the_answer:='SomeText' " />
//       <Bandit rb_name="{the_answer}"/>
//     </Sequence>
//   </BehaviorTree>
// </root>
// )";

// static const char* xml_tree_againn = R"(
// <root BTCPP_format="4">
//   <BehaviorTree ID="Untitled">
//     <Parallel failure_count="1"
//               success_count="-1">
//       <Sequence>
//         <Script code="mission_weight:=1.0"/>
//         <MissionNFR rob_position="{current_position}"
//                     weight="{mission_weight}">
//           <SLAMfd rob_position="{current_position}"/>
//         </MissionNFR>
//       </Sequence>
//       <IDfd/>
//     </Parallel>
//   </BehaviorTree>
// </root>
// )";
//in_voltage="{voltage}" in_temperature="{temperature}" in_current="{current}" in_charge="{charge}" in_capacity="{capacity}" in_design_capacity="{design_capacity}" in_percentage="{percentage}"

static const char* xml_tree = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Untitled">
    <Parallel failure_count="1"
              success_count="-1">
      <EnergyNFR weight="{energy_weight}" in_voltage="{voltage}" in_linear_speed="{linear_speed}" in_temperature="{temperature}" in_current="{current}" in_charge="{charge}" in_capacity="{capacity}" in_design_capacity="{design_capacity}" in_percentage="{percentage}" metric="{energy_metric}">
      <Sequence>
        <Script code="mission_weight:=1.0; current_position:=1.0; energy_weight=1.0; energy_metric:=1.0; mission_metric:=1.0" />
        <SLAMfd  rob_position="{current_position}" />
        <MissionNFR weight="{mission_weight}" rob_position="{current_position}" objs_identified="{objects_detected}" metric="{mission_metric}" mean_metric="{mission_mean_metric}" >
          <IDfd objs_identified="{objects_detected}" out_time_elapsed="{id_time_elapsed}" out_picture_rate="{id_picture_rate}"/>
        </MissionNFR>
      </Sequence>
      </EnergyNFR>
        <AlwaysSuccess />
    </Parallel>
  </BehaviorTree>
</root>
)";

static const char* xml_treeA = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Untitled">
    <Parallel failure_count="1"
              success_count="-1">
      <Sequence>
        <Script code="mission_weight:=1.0; current_position:=1.0" />
        <MissionNFR weight="{mission_weight}" rob_position="{current_position}" objs_identified="{objects_detected}" >
          <AlwaysSuccess />
        </MissionNFR>
      </Sequence>
          <SLAMfd  rob_position="{current_position}" />
    </Parallel>
  </BehaviorTree>
</root>
)";

// static const char* xml_tree_nonfr = R"(
// <root BTCPP_format="4">
//   <BehaviorTree ID="Untitled">
//     <Sequence>
//       <Script code=" safety_weight:=1.0; the_answer:='SomeText' " />
//         <Bandit rb_name="{the_answer}"/>
//     </Sequence>
//   </BehaviorTree>
// </root>
// )";


// static const char* bandit_xml2 = R"(
// <root BTCPP_format="4">
//   <BehaviorTree ID="Untitled">
//     <Sequence>
//       <Bandit rb_name="{the_answer}"/>
//     </Sequence>
//   </BehaviorTree>
// </root>
// )";


class Arborist : public rclcpp::Node
{
public:
  using SetBlackboard = altruism_msgs::srv::SetBlackboard;
  using GetBlackboard = altruism_msgs::srv::GetBlackboard;
  using GetNFR = altruism_msgs::srv::GetNFR;
  using SetWeights = altruism_msgs::srv::SetWeights;
  using NFR_MSG = altruism_msgs::msg::NFR;
  using BTAction = altruism_msgs::action::BehaviorTree;
  using GoalHandleBTAction = rclcpp_action::ServerGoalHandle<BTAction>;
  const std::string MSN_MAX_OBJ_PS_NAME = "max_objects_per_second";
  const std::string MSN_WINDOW_LEN_NAME = "mission_qa_window";

  BT::Tree tree;
  BehaviorTreeFactory factory;

  Arborist(std::string name = "arborist_node") : Node(name)
  {
      _set_blackboard = this->create_service<SetBlackboard>("set_blackboard", std::bind(&Arborist::handle_set_bb, this, _1, _2));
      _get_blackboard = this->create_service<GetBlackboard>("get_blackboard", std::bind(&Arborist::handle_get_bb, this, _1, _2));
      _get_nfr = this->create_service<GetNFR>("get_nfr", std::bind(&Arborist::handle_get_nfr, this, _1, _2));
      _set_weights = this->create_service<SetWeights>("set_weights", std::bind(&Arborist::handle_set_weights, this, _1, _2));

      

      //_start_tree = this->create_service<SetBlackboard>("set_blackboard", std::bind(&Arborist::handle_set_bb, this, _1, _2));
      _start_tree = rclcpp_action::create_server<BTAction>(
      this,
      "start_tree",
      std::bind(&Arborist::handle_tree_goal, this, _1, _2),
      std::bind(&Arborist::handle_tree_cancel, this, _1),
      std::bind(&Arborist::handle_tree_accepted, this, _1));

      registerCustomNode<BanditAction>(factory, "bt_bandit_client", "bandit", "Bandit");
      registerCustomNode<SLAMAction>(factory, "bt_slam_client", "slam", "SLAMfd");
      registerCustomNode<IdentifyAction>(factory, "bt_identify_client", "identify", "IDfd");


      factory.registerNodeType<MissionCompleteNFR>("MissionNFR");
      factory.registerNodeType<EnergyNFR>("EnergyNFR");

      tree = factory.createTreeFromText(xml_tree);

      this->declare_parameter(MSN_MAX_OBJ_PS_NAME, 1);
      this->declare_parameter(MSN_WINDOW_LEN_NAME, 20);

      int max_objs_ps = this->get_parameter(MSN_MAX_OBJ_PS_NAME).as_int();
      int window_length = this->get_parameter(MSN_WINDOW_LEN_NAME).as_int();


      auto mission_visitor = [max_objs_ps, window_length](TreeNode* node)
      {
        if (auto mission_nfr_node = dynamic_cast<MissionCompleteNFR*>(node))
        {
          mission_nfr_node->initialize(max_objs_ps,
                                       window_length);
        }
      };

      // Apply the visitor to ALL the nodes of the tree
      tree.applyVisitor(mission_visitor);
  }

  template <class T>
  void registerCustomNode(BehaviorTreeFactory& factory, std::string client_name, std::string action_name, std::string name_in_xml)
  {
    auto nh = std::make_shared<rclcpp::Node>(client_name);

    RosNodeParams params;
    params.nh = nh;
    params.default_port_value = action_name;

    factory.registerNodeType<T>(name_in_xml, params);

  }

private:

  bool inject_script_node(std::string script)
  {
    std::string _script;
    ScriptFunction _executor;

    auto executor = ParseScript(script);
    if (!executor)
    {
      return false;

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

    return true;
    
  }
  void handle_set_bb(const std::shared_ptr<SetBlackboard::Request> request,
        std::shared_ptr<SetBlackboard::Response> response)
  {
    std::cout<<"Set BlackBoard Service Called in Arborist Node"<<std::endl;

    std::string script = request->script_code;
    
    response->success = inject_script_node(script);
  }

  void handle_get_bb(const std::shared_ptr<GetBlackboard::Request> request,
        std::shared_ptr<GetBlackboard::Response> response)
  {
    std::cout << "Service to get a value from the blackboard" << std::endl;

    const std::string sought_key = request->key_name;

    std::cout << "sought key" << sought_key << std::endl;
    //this get isn't safe for the value not being assigned yet..
    auto entry_value = tree.rootBlackboard()->get<std::string>(sought_key);

    std::cout << "the value is " << entry_value << " gotten from the bb " << std::endl;

    response->key_value = entry_value;
  }

  std::vector<NFRNode*> get_tree_nfrs()
  {    
    std::vector<NFRNode*> nfr_nodes;

    // auto visitor = [nfr_nodes](TreeNode* node)
    // {
    //   if (auto nfr_node = dynamic_cast<NFRNode*>(node))
    //   {
    //     std::cout << "This new thing worked and I found the following node " << nfr_node->registrationName() << std::endl;
    //     nfr_nodes.push_back(nfr_node);

    //     //action_B_node->initialize(69, "interesting_value");
    //   }
    // };

    // // Apply the visitor to ALL the nodes of the tree
    // tree.applyVisitor(visitor);
    for (auto const & sbtree : tree.subtrees) 
    {
      for (auto & node : sbtree->nodes) 
      {
        if(auto nfr_node = dynamic_cast<NFRNode*>(static_cast<TreeNode*>(node.get())))
        {
          nfr_nodes.push_back(nfr_node);
        }
      }
    }
    return nfr_nodes;
  }
  
  void handle_get_nfr(const std::shared_ptr<GetNFR::Request> request,
        std::shared_ptr<GetNFR::Response> response)
  {
    auto nfr_nodes = get_tree_nfrs();
    for (auto & node : nfr_nodes) 
    {
      if(node->status() == NodeStatus::RUNNING) //ensures that the NFRs are currently in effect.
      {
      auto node_config = node->config();

      auto weight = node_config.input_ports.find(NFRNode::WEIGHT);
      auto metric = node_config.output_ports.find(NFRNode::METRIC);

      if (weight == node_config.input_ports.end() || metric == node_config.output_ports.end())
      {
        std::cout << "weight or metric not found" << std::endl;
        return;
      }

      auto metric_bb_value = tree.rootBlackboard()->get<double>((std::string)TreeNode::stripBlackboardPointer(metric->second));
      auto weight_bb_value = tree.rootBlackboard()->get<double>((std::string)TreeNode::stripBlackboardPointer(weight->second));

      NFR_MSG nfr_msg;
      nfr_msg.nfr_name = node->registrationName();
      nfr_msg.metric = metric_bb_value; 
      nfr_msg.weight = weight_bb_value;

      response->nfrs_in_tree.push_back(nfr_msg);
      }
    }
  }

  void handle_set_weights(const std::shared_ptr<SetWeights::Request> request,
        std::shared_ptr<SetWeights::Response> response)
  {
    std::string script = "";
    auto nfr_nodes = get_tree_nfrs();
    auto nfr_msgs = request->nfrs_to_update;


    for (NFR_MSG & nfr_msg : nfr_msgs)
    {
      std::string nfr_name = nfr_msg.nfr_name;
    
      for (auto & node : nfr_nodes) 
      {
        if(nfr_name == node->registrationName())
        {
          auto node_config = node->config();

          auto weight = node_config.input_ports.find(NFRNode::WEIGHT);

          if (weight == node_config.input_ports.end())
          {
            std::cout << "weight not found" << std::endl;
            return;
          }
          
          std::string weight_bb_key = (std::string)TreeNode::stripBlackboardPointer(weight->second);

          std::cout << (float)nfr_msg.weight << " uh " << std::endl;
          float weight_as_float = (float)nfr_msg.weight;
          script += weight_bb_key;
          script += ":=";
          script += std::to_string(nfr_msg.weight);
          script += "; ";
        }

      }

    }

    std::cout << "The script inside set_weights " << script << std::endl;
    script.pop_back();
    script.pop_back(); //Removing the last '; ' as it isn't necessary.

    response->success = inject_script_node(script);

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
          
          //Reporting on mission
          float id_time_elapsed = tree.rootBlackboard()->get<float>("id_time_elapsed");
          float id_picture_rate = tree.rootBlackboard()->get<float>("id_picture_rate");
          float avg_mission_metric = tree.rootBlackboard()->get<float>("mission_mean_metric");

          
          auto curr_time_pointer = std::chrono::system_clock::now();
          int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();

          // file pointer
          std::fstream fout;
        
          // opens an existing csv file or creates a new file.
          fout.open("altruism_results.csv", std::ios::out | std::ios::app);
        
          //Header
          // fout << "timestamp" << ", " 
          //     << "picture_rate" << ", "
          //     << "time_elapsed" << "\n";

          // Insert the data to file
          fout << current_time << ", " 
              << id_picture_rate << ", "
               << avg_mission_metric << ", "
              << id_time_elapsed << "\n";
            
          fout.close();

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
  rclcpp::Service<GetBlackboard>::SharedPtr _get_blackboard;
  rclcpp::Service<GetNFR>::SharedPtr _get_nfr;
  rclcpp::Service<SetWeights>::SharedPtr _set_weights;

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
