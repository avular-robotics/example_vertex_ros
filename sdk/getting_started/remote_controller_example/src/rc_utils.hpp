// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#pragma once

#include <creos/messages/controller_state.hpp>
#include <cstddef>
#include <string>
#include <vector>

class RcUtils
{
public:
    RcUtils()  = default;
    ~RcUtils() = default;

    std::string CreateControllerInputDisplay(const creos_messages::ControllerState &state);

private:
    std::vector<std::string> button_history_;
};
