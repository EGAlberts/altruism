#include "altruism/system_attribute_value.hpp"

#include <string>
#include <vector>

using altruism::SystemAttributeType;
using altruism::SystemAttributeValue;

std::string
altruism::to_string(const SystemAttributeType type)
{
  switch (type) {
    case SystemAttributeType::ATTRIBUTE_NOT_SET:
      return "not set";
    case SystemAttributeType::ATTRIBUTE_ODOM:
      return "odometry msg";
    default:
      return "unknown type";
  }
}

std::ostream &
altruism::operator<<(std::ostream & os, const SystemAttributeType type)
{
  os << altruism::to_string(type);
  return os;
}

template<typename ValType, typename PrintType = ValType>
std::string
array_to_string(
  const std::vector<ValType> & array,
  const std::ios::fmtflags format_flags = std::ios::dec)
{
  std::stringstream type_array;
  bool first_item = true;
  type_array << "[";
  type_array.setf(format_flags, std::ios_base::basefield | std::ios::boolalpha);
  type_array << std::showbase;
  for (const ValType & value : array) {
    if (!first_item) {
      type_array << ", ";
    } else {
      first_item = false;
    }
    type_array << static_cast<PrintType>(value);
  }
  type_array << "]";
  return type_array.str();
}

std::string
altruism::to_string(const SystemAttributeValue & value)
{
  switch (value.get_type()) {
    case SystemAttributeType::ATTRIBUTE_NOT_SET:
      return "not set";
    case SystemAttributeType::ATTRIBUTE_ODOM:
        return "odom message inside :) ";
    default:
      return "unknown type";
  }
}

SystemAttributeValue::SystemAttributeValue()
{
  value_.type = altruism_msgs::msg::SystemAttributeType::ATTRIBUTE_NOT_SET;
}

SystemAttributeValue::SystemAttributeValue(const altruism_msgs::msg::SystemAttributeValue & value)
{
  value_ = value;
  switch (value.type) {
    case ATTRIBUTE_ODOM:
    case ATTRIBUTE_NOT_SET:
      break;
    default:
      throw std::runtime_error(std::string("Unknown type encountered when trying to construct SystemAttributeValue"));
  }
}

SystemAttributeValue::SystemAttributeValue(const nav_msgs::msg::Odometry odom_value)
{
  value_.odom_value = odom_value;
  value_.type = altruism_msgs::msg::SystemAttributeType::ATTRIBUTE_ODOM;
}

SystemAttributeType
SystemAttributeValue::get_type() const
{
  return static_cast<SystemAttributeType>(value_.type);
}

altruism_msgs::msg::SystemAttributeValue
SystemAttributeValue::to_value_msg() const
{
  return value_;
}

bool
SystemAttributeValue::operator==(const SystemAttributeValue & rhs) const
{
  return this->value_ == rhs.value_;
}

bool
SystemAttributeValue::operator!=(const SystemAttributeValue & rhs) const
{
  return this->value_ != rhs.value_;
}