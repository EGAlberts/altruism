cmake_minimum_required(VERSION 3.8)
project(altruism_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Bandit.action"
  "action/BehaviorTree.action"
  "action/SLAM.action"
  "action/Identify.action"
  "srv/SetBlackboard.srv"
  "srv/GetBlackboard.srv"
  "srv/GetNFR.srv"
  "srv/GetVariableParams.srv"
  "srv/SetWeights.srv"
  "msg/ObjectsIdentified.msg"
  "msg/NFR.msg"
  "msg/VariableParameter.msg"
  "msg/VariableParameters.msg"


  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs rcl_interfaces
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
