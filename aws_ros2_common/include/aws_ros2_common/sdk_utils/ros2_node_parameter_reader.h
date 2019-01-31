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
#pragma once
#include <aws_common/sdk_utils/parameter_reader.h>
#include <rclcpp/rclcpp.hpp>

namespace Aws {
namespace Client {

/**
 * ROS2-specific parameter reader.
 * @note Currently, only the reading of parameters local to the node is supported.
 */
class Ros2NodeParameterReader : public ParameterReaderInterface
{
public:
    Ros2NodeParameterReader(const std::weak_ptr<rclcpp::Node> node) : node_(node) {}

    using ParameterReaderInterface::ReadList;
    using ParameterReaderInterface::ReadDouble;
    using ParameterReaderInterface::ReadInt;
    using ParameterReaderInterface::ReadBool;
    using ParameterReaderInterface::ReadStdString;
    using ParameterReaderInterface::ReadString;
    using ParameterReaderInterface::ReadMap;

    AwsError ReadList(const char * name, std::vector<std::string> & out) const override;
    AwsError ReadDouble(const char * name, double & out) const override;
    AwsError ReadInt(const char * name, int & out) const override;
    AwsError ReadBool(const char * name, bool & out) const override;
    AwsError ReadStdString(const char * name, std::string & out) const override;
    AwsError ReadString(const char * name, Aws::String & out) const override;
    AwsError ReadMap(const char * name, std::map<std::string, std::string> & out) const override;

private:
    std::string FormatParameterPath(const ParameterPath & param_path) const override;

    /**
     * Generic parameter reading helper function.
     * @param parameter_path object representing the parameter's path
     * @param out reference to which the parameter's value should be written to.
     */
    template <class T>
    AwsError ReadParam(const char * name, T & out) const;

    const std::weak_ptr<rclcpp::Node> node_;
};

} /* namespace */
} /* namespace */
