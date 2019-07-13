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
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>

#define CLIENT_CONFIG_PREFIX        "aws_client_configuration"

/**
 * Populates a ROS node and a client configuration with the same dummy values.
 * @param node
 * @param config
 */
void initialize_node_and_config(rclcpp::Node::SharedPtr node, Aws::Client::ClientConfiguration &config)
{
    rclcpp::Parameter region(CLIENT_CONFIG_PREFIX ".region", "uk-north-20");
    node->declare_parameter(CLIENT_CONFIG_PREFIX ".region");
    config.region = "uk-north-20";
    rclcpp::Parameter proxyPort(CLIENT_CONFIG_PREFIX ".proxy_port", 787);
    node->declare_parameter(CLIENT_CONFIG_PREFIX ".proxy_port");
    config.proxyPort = 787;
    rclcpp::Parameter connectTimeoutMs(CLIENT_CONFIG_PREFIX ".connect_timeout_ms", 511111);
    node->declare_parameter(CLIENT_CONFIG_PREFIX ".connect_timeout_ms");
    config.connectTimeoutMs = 511111;
    rclcpp::Parameter verifySSL(CLIENT_CONFIG_PREFIX ".verify_SSL", true);
    node->declare_parameter(CLIENT_CONFIG_PREFIX ".verify_SSL");
    config.verifySSL = true;
    rclcpp::Parameter followRedirects(CLIENT_CONFIG_PREFIX ".follow_redirects", true);
    node->declare_parameter(CLIENT_CONFIG_PREFIX ".follow_redirects");
    config.followRedirects = true;
    node->set_parameters({region, proxyPort, connectTimeoutMs, verifySSL, followRedirects});
}

/**
 * Tests that GetClientConfiguration returns the expected ClientConfiguration object.
 */
TEST(DefaultClientConfigurationProvider, getClientConfiguration)
{
    Aws::Client::ClientConfiguration prepared_config;
    auto dummy_node = rclcpp::Node::make_shared("_");
    auto parameter_reader = std::make_shared<Aws::Client::Ros2NodeParameterReader>(dummy_node);
    Aws::Client::ClientConfigurationProvider config_provider(parameter_reader);

    initialize_node_and_config(dummy_node, prepared_config);
    Aws::Client::ClientConfiguration generated_config = config_provider.GetClientConfiguration();
    prepared_config.userAgent = generated_config.userAgent; /* Set the user agent to w/e was generated. Will be tested separately */

    prepared_config.followRedirects = false;
    ASSERT_NE(prepared_config, generated_config);

    prepared_config.followRedirects = true;
    ASSERT_EQ(prepared_config, generated_config);
}

/**
 * Tests that the configuration provider sets userAgent correctly with the ROS distro & version information.
 */
TEST(DefaultClientConfigurationProvider, userAgentTest)
{
    Aws::Client::ClientConfiguration prepared_config;
    auto dummy_node = rclcpp::Node::make_shared("_");
    auto parameter_reader = std::make_shared<Aws::Client::Ros2NodeParameterReader>(dummy_node);
    Aws::Client::ClientConfigurationProvider config_provider(parameter_reader);
    initialize_node_and_config(dummy_node, prepared_config);

    /* Verify userAgent starts with "ros-" */
    Aws::Client::ClientConfiguration generated_config = config_provider.GetClientConfiguration();
    size_t ros_user_agent_index = generated_config.userAgent.find(std::string("exec-env/AWS_RoboMaker ros-").c_str());
    ASSERT_NE(std::string::npos, ros_user_agent_index);

    /* Test version override */
    const char version_override[] = "1.12.16";
    generated_config = config_provider.GetClientConfiguration(version_override);
    ASSERT_EQ(0, generated_config.userAgent.substr(generated_config.userAgent.size() - (sizeof(version_override) - 1), sizeof(version_override) - 1).compare(version_override));

    /* Verify that we only add information on top of the SDK's userAgent */
    Aws::Client::ClientConfiguration stock_config;
    ASSERT_EQ(0, generated_config.userAgent.substr(0, ros_user_agent_index - 1).compare(stock_config.userAgent));
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    return RUN_ALL_TESTS();
}
