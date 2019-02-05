cmake_minimum_required(VERSION 3.5)
project(aws_ros2_common)

add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aws_common REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

if(AWSSDK_FOUND)
  set(SERVICE core)
  AWSSDK_DETERMINE_LIBS_TO_LINK(SERVICE AWSSDK_LIBS)
  link_directories("${AWSSDK_LIB_DIR}")
endif()


include_directories(include ${rclcpp_INCLUDE_DIRS} ${aws_common_INCLUDE_DIRS})
add_library(${PROJECT_NAME} SHARED
    src/sdk_utils/ros2_node_parameter_reader.cpp
    src/sdk_utils/logging/aws_ros_logger.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${AWSSDK_INCLUDE_DIR})
ament_target_dependencies(${PROJECT_NAME} aws_common)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include/
)

ament_add_gtest(test_configuration_provider test/client_configuration_provider_test.cpp)
target_link_libraries(test_configuration_provider
    ${aws_common_LIBRARIES}
    ${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
)

ament_add_gtest(test_logger test/sdk_utils/logging/aws_ros_logger_test.cpp)
target_link_libraries(test_logger
    ${aws_common_LIBRARIES}
    ${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
)

ament_add_gtest(test_parameter_reader test/parameter_reader_test.cpp)
target_link_libraries(test_parameter_reader
    ${aws_common_LIBRARIES}
    ${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories("include")
ament_package()
