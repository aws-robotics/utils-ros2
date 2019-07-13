/*
* Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License").
* You may not use this file except in compliance with the License.
* A copy of the License is located at
*
*  http://aws.amazon.com/apache2.0
*
* or in the "license" file accompanying this file. This file is distributed
* on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
* express or implied. See the License for the specific language governing
* permissions and limitations under the License.
*/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <aws/core/client/ClientConfiguration.h>

#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_common/sdk_utils/aws_error.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>

using namespace Aws::Client;
using namespace std::chrono_literals;


#define PARAM_READER_TEST__PARAM_PREFIX "configuration_namespace9181"
#define PARAM_READER_TEST__PARAM_KEY "someBogusParamKey"
#define PARAM_READER_TEST__PARAM_VALUE "uk-north-2180"


TEST(ParameterReader, parameterPathResolution)
{
    Aws::Client::ClientConfiguration prepared_config;
    auto dummy_node = rclcpp::Node::make_shared("_");
    dummy_node->declare_parameter(PARAM_READER_TEST__PARAM_PREFIX "." PARAM_READER_TEST__PARAM_KEY);
    auto parameter_reader = std::make_shared<Aws::Client::Ros2NodeParameterReader>(dummy_node);
    dummy_node->set_parameters(
        {rclcpp::Parameter(PARAM_READER_TEST__PARAM_PREFIX "." PARAM_READER_TEST__PARAM_KEY, PARAM_READER_TEST__PARAM_VALUE)}
    );

    auto param_path_flat_ros1_style = ParameterPath(PARAM_READER_TEST__PARAM_PREFIX "/" PARAM_READER_TEST__PARAM_KEY);
    auto param_path_flat_ros2_style = ParameterPath(PARAM_READER_TEST__PARAM_PREFIX "." PARAM_READER_TEST__PARAM_KEY);
    auto param_path_variadic = ParameterPath(PARAM_READER_TEST__PARAM_PREFIX, PARAM_READER_TEST__PARAM_KEY);
    auto param_path_complex = ParameterPath(std::vector<std::string>{},
        std::vector<std::string>{PARAM_READER_TEST__PARAM_PREFIX, PARAM_READER_TEST__PARAM_KEY});

    ASSERT_NE(param_path_flat_ros1_style.get_resolved_path('/', '.'), param_path_variadic.get_resolved_path('/', '.'));
    ASSERT_EQ(param_path_flat_ros2_style.get_resolved_path('/', '.'), param_path_variadic.get_resolved_path('/', '.'));

    std::string flat_ros1_style_result;
    parameter_reader->ReadParam(param_path_flat_ros1_style, flat_ros1_style_result);
    std::string flat_ros2_style_result;
    parameter_reader->ReadParam(param_path_flat_ros2_style, flat_ros2_style_result);
    std::string variadic_result;
    parameter_reader->ReadParam(param_path_variadic, variadic_result);
    std::string complex_result;
    parameter_reader->ReadParam(param_path_complex, complex_result);

    ASSERT_NE(flat_ros1_style_result, flat_ros2_style_result);
    ASSERT_EQ(flat_ros2_style_result, variadic_result);
    ASSERT_EQ(variadic_result, complex_result);
    ASSERT_EQ(complex_result, std::string(PARAM_READER_TEST__PARAM_VALUE));

    std::string stored_complex_result = complex_result;
    ASSERT_EQ(Aws::AwsError::AWS_ERR_NOT_SUPPORTED, parameter_reader->ReadParam(
              ParameterPath({"some_ns", "some_other_node"}, {"parameter"}), complex_result));
    ASSERT_EQ(stored_complex_result, complex_result);
}

TEST(ParameterReader, failureTests)
{
    auto dummy_node = rclcpp::Node::make_shared("_");
    auto parameter_reader = std::make_shared<Aws::Client::Ros2NodeParameterReader>(dummy_node);

    auto nonexistent_path = ParameterPath("I don't exist");
    std::string nonexistent_path_result = PARAM_READER_TEST__PARAM_VALUE;
    /* Querying for a nonexistent parameter should return NOT_FOUND and the out parameter remains unchanged. */
    ASSERT_EQ(Aws::AwsError::AWS_ERR_NOT_FOUND, parameter_reader->ReadParam(nonexistent_path, nonexistent_path_result));
    ASSERT_EQ(nonexistent_path_result, std::string(PARAM_READER_TEST__PARAM_VALUE));

    dummy_node.reset();
    /* Using a deleted node should return a memory error and the out parameter remains unchanged. */
    ASSERT_EQ(Aws::AwsError::AWS_ERR_MEMORY, parameter_reader->ReadParam(nonexistent_path, nonexistent_path_result));
    ASSERT_EQ(nonexistent_path_result, std::string(PARAM_READER_TEST__PARAM_VALUE));

    dummy_node = rclcpp::Node::make_shared("_");
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    return RUN_ALL_TESTS();
}
