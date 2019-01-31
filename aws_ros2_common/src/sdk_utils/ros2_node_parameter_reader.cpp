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

#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>
#include <rclcpp/rclcpp.hpp>

namespace Aws {
namespace Client {

constexpr char kParameterNsSeparator = '.';
constexpr char kNodeNsSeparator = '/';

/**
 * Generic parameter reading helper function.
 * @param parameter_path object representing the parameter's path
 * @param out reference to which the parameter's value should be written to.
 */
template <class T>
AwsError Ros2NodeParameterReader::ReadParam(const char * name, T & out) const {
    if (nullptr != strchr(name, kNodeNsSeparator)) {
        /* Reading remote node's parameters is not supported yet by Ros2NodeParameterReader. */
        return AWS_ERR_NOT_SUPPORTED;
    }
    if (auto node_handle = node_.lock()) {
        if (node_handle->get_parameter(name, out)) {
            return AWS_ERR_OK;
        } else {
            return AWS_ERR_NOT_FOUND;
        }
    } else {
        /* The node object has been destroyed. */
        return AWS_ERR_MEMORY;
    }
}

AwsError Ros2NodeParameterReader::ReadList(const char * name, std::vector<std::string> &out) const {
    return ReadParam(name, out);
}

AwsError Ros2NodeParameterReader::ReadDouble(const char * name, double &out) const {
    return ReadParam(name, out);
}

AwsError Ros2NodeParameterReader::ReadInt(const char * name, int &out) const {
    return ReadParam(name, out);
}

AwsError Ros2NodeParameterReader::ReadBool(const char * name, bool &out) const {
    return ReadParam(name, out);
}

AwsError Ros2NodeParameterReader::ReadStdString(const char * name, std::string &out) const {
    return ReadParam(name, out);
}

AwsError Ros2NodeParameterReader::ReadString(const char * name, Aws::String &out) const {
    std::string value;
    AwsError result = ReadStdString(name, value);
    if (result == AWS_ERR_OK) {
        out = Aws::String(value.c_str());
    }
    return result;
}

AwsError Ros2NodeParameterReader::ReadMap(const char * name, std::map<std::string, std::string> & out) const {
    return AWS_ERR_NOT_SUPPORTED;
}

std::string Ros2NodeParameterReader::FormatParameterPath(const ParameterPath & param_path) const {
    return param_path.get_resolved_path(kNodeNsSeparator, kParameterNsSeparator);
}

} /* namespace */
} /* namespace */
