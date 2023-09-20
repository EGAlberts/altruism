#ifndef ALTRUISM__PARAMETER_VALUE_HPP_
#define ALTRUISM__PARAMETER_VALUE_HPP_

#include <exception>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "altruism_msgs/msg/system_attribute_value.hpp"
#include "altruism_msgs/msg/system_attribute_type.hpp"

#include "nav_msgs/msg/odometry.hpp"


//Inspired by rclcpp/parameter_value.hpp

namespace altruism
{

enum SystemAttributeType : uint8_t
{
  ATTRIBUTE_NOT_SET = altruism_msgs::msg::SystemAttributeType::ATTRIBUTE_NOT_SET,
  ATTRIBUTE_ODOM    = altruism_msgs::msg::SystemAttributeType::ATTRIBUTE_ODOM,
};

/// Return the name of a parameter type
std::string
to_string(SystemAttributeType type);

std::ostream &
operator<<(std::ostream & os, SystemAttributeType type);

class SystemAttributeTypeException : public std::runtime_error
{
public:
  /// Construct an instance.
  /**
   * \param[in] expected the expected parameter type.
   * \param[in] actual the actual parameter type.
   */
  SystemAttributeTypeException(SystemAttributeType expected, SystemAttributeType actual)
  : std::runtime_error("expected [" + to_string(expected) + "] got [" + to_string(actual) + "]")
  {}
};

/// A class wrapper around the SystemAttributeValue MSG we've defined
class SystemAttributeValue
{
public:
  /// Construct a parameter value with type PARAMETER_NOT_SET.
  
  SystemAttributeValue();
  
  /// Construct a parameter value from a message.
  explicit SystemAttributeValue(const altruism_msgs::msg::SystemAttributeValue & value);
  /// Construct a parameter value with type PARAMETER_BOOL.
  explicit SystemAttributeValue(const nav_msgs::msg::Odometry odom_value);

  /// Return an enum indicating the type of the set value.
  
  SystemAttributeType
  get_type() const;

  /// Return a message populated with the parameter value
  
  altruism_msgs::msg::SystemAttributeValue
  to_value_msg() const;

  /// Equal operator.
  
  bool
  operator==(const SystemAttributeValue & rhs) const;

  /// Not equal operator.
  
  bool
  operator!=(const SystemAttributeValue & rhs) const;

  // The following get() variants require the use of SystemAttributeType

  template<SystemAttributeType type>
  constexpr
  typename std::enable_if<type == SystemAttributeType::ATTRIBUTE_ODOM, const bool &>::type
  get() const
  {
    if (value_.type != altruism_msgs::msg::SystemAttributeType::ATTRIBUTE_ODOM) {
      throw SystemAttributeTypeException(SystemAttributeType::ATTRIBUTE_ODOM, get_type());
    }
    return value_.odom_value;
  }

  
private:
  altruism_msgs::msg::SystemAttributeValue value_;
};

/// Return the value of a parameter as a string

std::string
to_string(const SystemAttributeValue & type);


} //namespace altruism

#endif  // ALTRUISM__PARAMETER_VALUE_HPP_