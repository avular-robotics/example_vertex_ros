// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#pragma once

#include <sensor_msgs/msg/joy.hpp>

#include <string>
#include <vector>

class RcUtils
{
public:
    RcUtils()  = default;
    ~RcUtils() = default;

    std::string CreateControllerInputDisplay(const sensor_msgs::msg::Joy &state);

private:
    std::vector<std::string> button_history_;
};
