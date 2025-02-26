// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#pragma once

#include <functional>
#include <creos/messages/controller_state.hpp>
#include <iostream>

class IRemoteController
{
public:
    virtual ~IRemoteController(){};

    virtual void RegisterActivationButtonCallback(std::function<void()> callback) = 0;
    virtual std::function<void(const creos_messages::ControllerState &)>
                 GetControllerStateCallback() = 0;
    virtual void ControllerStateReceivedCallback(
        const creos_messages::ControllerState &controller_state) = 0;
};

enum class ControllerType
{
    kHerelink,
    kJeti
};

extern std::shared_ptr<IRemoteController> CreateRemoteController(ControllerType type);
