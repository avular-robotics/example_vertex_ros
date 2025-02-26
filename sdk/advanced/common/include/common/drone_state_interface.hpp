// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#pragma once

#include <creos/messages/control_source.hpp>

class IDroneState
{
public:
    virtual ~IDroneState() = default;

    virtual double                               GetYaw()           = 0;
    virtual const creos_messages::ControlSource &GetControlSource() = 0;

    virtual bool IsArmed()                     = 0;
    virtual bool IsDisarmed()                  = 0;
    virtual bool IsPerformingPreFlightChecks() = 0;
    virtual bool IsReadyForTakeOff()           = 0;
    virtual bool IsInTakeOff()                 = 0;
    virtual bool IsInFlight()                  = 0;
    virtual bool IsLanding()                   = 0;
    virtual bool IsInUserControlMode()         = 0;
};
