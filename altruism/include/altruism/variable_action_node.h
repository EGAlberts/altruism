#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "altruism_msgs/msg/variable_parameters.hpp"
#include "altruism_msgs/msg/variable_parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

using namespace BT;
using VariableParameters = altruism_msgs::msg::VariableParameters;
using ParamValue = rcl_interfaces::msg::ParameterValue;


class VariableActionNodeBase {
public:
    static constexpr const char* VARIABLE_PARAMS = "variable_parameters";
};


template <class ActionT>
class VariableActionNode : public RosActionNode<ActionT>, public VariableActionNodeBase
{
public:
    VariableActionNode(const std::string &name, const BT::NodeConfig &config,const RosNodeParams &params)
        : RosActionNode<ActionT>(name, config, params)
    {
    }

    template <class T>
    void registerAdaptations(std::vector<T> param_values, std::string param_name)
    {
        altruism_msgs::msg::VariableParameter variable_param = altruism_msgs::msg::VariableParameter();

        variable_param.name = param_name;

        for (T val : param_values) {
            rclcpp::ParameterValue par_val = rclcpp::ParameterValue(val);
            variable_param.possible_values.push_back(par_val.to_value_msg());
        }
        _var_params.variable_parameters.push_back(variable_param); //vector of VariableParameter
        
    }


    static PortsList providedPorts()
    {
        return RosActionNode<ActionT>::providedBasicPorts({OutputPort<VariableParameters>(VARIABLE_PARAMS, "What can be changed at runtime about this action")});
    }

protected:
    VariableParameters _var_params = VariableParameters();


};

