// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include "rc_utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>

namespace
{
constexpr size_t kButtonHistorySize = 20;

std::string formatAxisBar(float value)
{
    constexpr int kHalfWidth = 10;
    value                    = std::clamp(value, -1.0F, 1.0F);
    const int marker_index   = static_cast<int>(std::round((value + 1.0F) * kHalfWidth));

    std::string bar(2 * kHalfWidth + 1, '.');
    bar[kHalfWidth]   = '|';
    bar[marker_index] = '#';
    return bar;
}

char buttonStateMarker(creos_messages::ControllerState::ButtonState state)
{
    using ButtonState = creos_messages::ControllerState::ButtonState;

    if(state == ButtonState::kLongPress)
    {
        return '#';
    }
    if(state == ButtonState::kDown)
    {
        return 'v';
    }
    if(state == ButtonState::kPressed || state == ButtonState::kUp)
    {
        return '*';
    }

    return '.';
}

std::string axisLabel(size_t index)
{
    switch(index)
    {
    case 0:
        return "roll / right stick X";
    case 1:
        return "pitch / right stick Y";
    case 2:
        return "throttle / left stick Y";
    case 3:
        return "yaw / left stick X";
    default:
        return "extra";
    }
}
} // namespace

std::string RcUtils::CreateControllerInputDisplay(
    const creos_messages::ControllerState &state)
{
    std::ostringstream output;
    output << "\nRemote controller input\n";
    output << "Axes:\n";

    for(size_t index = 0; index < state.axes.size(); ++index)
    {
        output << "  axis[" << index << "] " << std::showpos << std::fixed << std::setprecision(2)
               << state.axes[index] << std::noshowpos << " [" << formatAxisBar(state.axes[index])
               << "] " << axisLabel(index) << "\n";
    }

    output << "Buttons:\n";
    button_history_.resize(state.buttons.size(), std::string(kButtonHistorySize, '.'));

    for(size_t index = 0; index < state.buttons.size(); ++index)
    {
        auto &history = button_history_[index];
        history.erase(0, 1);
        history.push_back(buttonStateMarker(state.buttons[index]));

        output << "  button[" << index << "] [" << history << "]\n";
    }

    return output.str();
}
