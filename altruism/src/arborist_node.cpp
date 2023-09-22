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
#include "altruism_msgs/srv/set_attribute_in_blackboard.hpp"
#include "altruism_msgs/srv/get_blackboard.hpp"
#include "altruism_msgs/srv/get_nfr.hpp"
#include "altruism_msgs/srv/get_variable_params.hpp"
#include "altruism_msgs/srv/set_weights.hpp"
#include "altruism_msgs/msg/nfr.hpp"
#include "altruism_msgs/msg/system_attribute_value.hpp"




#include "altruism/system_attribute_value.hpp"
#include "altruism/bandit_action_node.h"
#include "altruism/slam_action_node.h"
#include "altruism/identify_action_node.h"


#include "altruism/nfr_node.h"

#include <fstream>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>


using namespace BT;
using std::placeholders::_1;
using std::placeholders::_2;




class Arborist : public rclcpp::Node
{
public:
  using SystemAttributeValueMsg = altruism_msgs::msg::SystemAttributeValue;
  using SetAttributeInBlackboard = altruism_msgs::srv::SetAttributeInBlackboard;
  using SetBlackboard = altruism_msgs::srv::SetBlackboard;
  using GetBlackboard = altruism_msgs::srv::GetBlackboard;
  using GetNFR = altruism_msgs::srv::GetNFR;
  using GetVParams = altruism_msgs::srv::GetVariableParams;
  using SetWeights = altruism_msgs::srv::SetWeights;
  using NFR_MSG = altruism_msgs::msg::NFR;
  using BTAction = altruism_msgs::action::BehaviorTree;
  using GoalHandleBTAction = rclcpp_action::ServerGoalHandle<BTAction>;
  const std::string MSN_MAX_OBJ_PS_NAME = "max_objects_per_second";
  const std::string ENG_MAX_PIC_PS_NAME = "max_pictures_per_second";
  const std::string MSN_WINDOW_LEN_NAME = "mission_qa_window";
  const std::string ENG_WINDOW_LEN_NAME = "energy_qa_window";

  const std::string BT_NAME_PARAM = "bt_filename";
  const std::string EXP_NAME_PARAM = "experiment_name";

  

  BT::Tree tree;
  BehaviorTreeFactory factory;
  std::string bt_name;
  std::string experiment_name;
  
  Arborist(std::string name = "arborist_node") : Node(name)
  { 

      _set_att_in_blackboard = this->create_service<SetAttributeInBlackboard>("set_attribute_in_blackboard", std::bind(&Arborist::handle_set_atb_bb, this, _1, _2));
      _set_blackboard = this->create_service<SetBlackboard>("set_blackboard", std::bind(&Arborist::handle_set_bb, this, _1, _2));
      _get_blackboard = this->create_service<GetBlackboard>("get_blackboard", std::bind(&Arborist::handle_get_bb, this, _1, _2));
      _get_nfr = this->create_service<GetNFR>("get_nfr", std::bind(&Arborist::handle_get_nfr, this, _1, _2));
      _get_var_param = this->create_service<GetVParams>("get_variable_params", std::bind(&Arborist::handle_get_var_params, this, _1, _2));
      _set_weights = this->create_service<SetWeights>("set_weights", std::bind(&Arborist::handle_set_weights, this, _1, _2));

      // may throw ament_index_cpp::PackageNotFoundError exception
      std::string tree_dir = ament_index_cpp::get_package_share_directory("altruism") + "/trees/";
      std::cout << tree_dir << std::endl;

      //_start_tree = this->create_service<SetBlackboard>("set_blackboard", std::bind(&Arborist::handle_set_bb, this, _1, _2));
      _start_tree = rclcpp_action::create_server<BTAction>(
      this,
      "start_tree",
      std::bind(&Arborist::handle_tree_goal, this, _1, _2),
      std::bind(&Arborist::handle_tree_cancel, this, _1),
      std::bind(&Arborist::handle_tree_accepted, this, _1));

      //I suppose here you register all the possible custom nodes, and the determination as to whether they are actually used lies in the xml tree provided.
      registerActionClient<BanditAction>(factory, "bt_bandit_client", "bandit", "Bandit");
      registerActionClient<SLAMAction>(factory, "bt_slam_client", "slam", "SLAMfd");
      registerActionClient<IdentifyAction>(factory, "bt_identify_client", "identify", "IDfd");
      factory.registerNodeType<MissionCompleteNFR>("MissionNFR");
      factory.registerNodeType<EnergyNFR>("EnergyNFR");




      this->declare_parameter(BT_NAME_PARAM, "onlyIDBandit.xml");
      this->declare_parameter(EXP_NAME_PARAM, "no_experiment_name");

      bt_name = this->get_parameter(BT_NAME_PARAM).as_string();
      experiment_name = this->get_parameter(EXP_NAME_PARAM).as_string();

      RCLCPP_INFO(this->get_logger(), bt_name.c_str());

      tree = factory.createTreeFromFile(tree_dir + bt_name);

      this->declare_parameter(MSN_MAX_OBJ_PS_NAME, 1.0);
      this->declare_parameter(ENG_MAX_PIC_PS_NAME, 1.0);
      this->declare_parameter(MSN_WINDOW_LEN_NAME, 20);
      this->declare_parameter(ENG_WINDOW_LEN_NAME, 20);
      
      


      double max_objs_ps = this->get_parameter(MSN_MAX_OBJ_PS_NAME).as_double();

      float max_pics_ps = this->get_parameter(ENG_MAX_PIC_PS_NAME).as_double();
      int msn_window_length = this->get_parameter(MSN_WINDOW_LEN_NAME).as_int();
      int eng_window_length = this->get_parameter(ENG_WINDOW_LEN_NAME).as_int();



      auto node_visitor = [max_objs_ps, msn_window_length, eng_window_length, max_pics_ps](TreeNode* node)
      {
        if (auto mission_nfr_node = dynamic_cast<MissionCompleteNFR*>(node))
        {
          mission_nfr_node->initialize(max_objs_ps,
                                       msn_window_length);
        }
        if (auto energy_nfr_node = dynamic_cast<EnergyNFR*>(node))
        {
          energy_nfr_node->initialize(max_pics_ps,
                                       eng_window_length);
        }
      };

      // Apply the visitor to ALL the nodes of the tree
      tree.applyVisitor(node_visitor);

  }

  template <class T>
  void registerActionClient(BehaviorTreeFactory& factory, std::string client_name, std::string action_name, std::string name_in_xml)
  {
    auto nh = std::make_shared<rclcpp::Node>(client_name);

    RosNodeParams params;
    params.nh = nh;
    params.default_port_value = action_name;
    params.server_timeout = std::chrono::milliseconds(4000); //This resolves a race condition with the ServerGoalTimeout (Error 1 in RosActionNode) I suppose caused by the size of the system.
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



  void handle_set_atb_bb(const std::shared_ptr<SetAttributeInBlackboard::Request> request,
        std::shared_ptr<SetAttributeInBlackboard::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Set Attribute Service Called in Arborist Node");

    auto sys_attr = request->sys_attribute;

    auto sys_attvalue_obj = altruism::SystemAttributeValue(sys_attr.value);
    tree.rootBlackboard()->set<altruism::SystemAttributeValue>(sys_attr.name, sys_attvalue_obj);
    

    //Call template func here
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

  std::vector<TreeNode::Ptr> get_tree_variable_acs()
  {    
    std::vector<TreeNode::Ptr> variable_action_nodes;

    for (auto const & sbtree : tree.subtrees) 
    {
      for (auto & node : sbtree->nodes) 
      {
        auto node_config = node->config();

        if(node->type() == NodeType::ACTION && (node_config.output_ports.find(VariableActionNodeBase::VARIABLE_PARAMS) != node_config.output_ports.end()))
        {
          variable_action_nodes.push_back(node);
        }
        // if(auto var_ac_node = dynamic_cast<ActionNodeBase*>(static_cast<TreeNode*>(node.get())))
        // {
        //   variable_action_nodes.push_back(var_ac_node);
        // }
      }
    }
    return variable_action_nodes;
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
        std::stringstream ss;
        ss << "Either port weight port " << NFRNode::WEIGHT << " or metric port " << NFRNode::METRIC << " not found within the QR node " << node->registrationName();
        RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
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

  void handle_get_var_params(const std::shared_ptr<GetVParams::Request> request,
        std::shared_ptr<GetVParams::Response> response)
  {
    auto var_nodes = get_tree_variable_acs();
    for (auto & node : var_nodes) 
    {
      if(node->status() == NodeStatus::RUNNING) //ensures that the actions are currently in effect.
      {
      auto node_config = node->config();

      auto var_params = node_config.output_ports.find(VariableActionNodeBase::VARIABLE_PARAMS); //this is safe because the previous method guarantees the presence of this port.

      // if (var_params == node_config.output_ports.end())
      // {
      //   std::stringstream ss;
      //   ss << "Output port " << VariableActionNode<ActionT>::VARIABLE_PARAMS << " not found within the variable action node " << node->registrationName();
      //   RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
      //   return;
      // }

      auto var_bb_value = tree.rootBlackboard()->get<VariableParameters>((std::string)TreeNode::stripBlackboardPointer(var_params->second));
      //Each variable action node can provide one or more changeable parameters.

      for (auto var_param : var_bb_value.variable_parameters)
      {
        response->variables_in_tree.variable_parameters.push_back(var_param); //We put all of these parameters that may change into a singular list. 
        //Potentially we may want a list of these lists to keep them distinct.. 
      }
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
          int32_t id_picture_rate = tree.rootBlackboard()->get<int32_t>("id_picture_rate");
          float avg_mission_metric = tree.rootBlackboard()->get<float>("mission_mean_metric");
          float avg_energy_metric = tree.rootBlackboard()->get<float>("energy_mean_metric");

          int32_t id_det_threshold = tree.rootBlackboard()->get<int32_t>("id_det_threshold");

          
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
              << avg_energy_metric << ", "
              << id_time_elapsed << ", "
              << id_det_threshold << ", "
              << experiment_name << ", "
              << bt_name << "\n";
            
          fout.close();



          std::ofstream outfile ("mission.done");

          outfile << "." << std::endl;

          outfile.close();

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
  rclcpp::Service<GetVParams>::SharedPtr _get_var_param;
  rclcpp::Service<SetWeights>::SharedPtr _set_weights;
  rclcpp::Service<SetAttributeInBlackboard>::SharedPtr _set_att_in_blackboard;


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
