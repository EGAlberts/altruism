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

