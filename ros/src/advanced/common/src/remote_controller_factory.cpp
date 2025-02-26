// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include <common/remote_controller_interface.hpp>

#include "herelink_remote.hpp"
#include "jeti_remote.hpp"

std::shared_ptr<IRemoteController> CreateRemoteController(ControllerType type,
                                                          rclcpp::Logger logger)
{
    switch(type)
    {
    case ControllerType::kHerelink:
        return std::make_shared<HereLinkRemote>(logger);
    case ControllerType::kJeti:
        return std::make_shared<JetiRemote>(logger);
    default:
        throw std::runtime_error("Controller type not supported");
    }
}
