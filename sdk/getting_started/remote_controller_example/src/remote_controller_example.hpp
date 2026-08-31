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

#include "rc_utils.hpp"

#include <spdlog/spdlog.h>

inline void handleRemoteControllerInput(
    const creos_messages::ControllerState &state,
    RcUtils                               &rc_utils)
{
    spdlog::info("{}", rc_utils.CreateControllerInputDisplay(state));
}
