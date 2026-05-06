#pragma once

#include <creos/autopilot/navigation.hpp>

#include <functional>
#include <mutex>

class CoordinateManager
{
public:
	std::function<void(const creos_messages::CoordinateSystemInfo &)> GetCoordinateSystemInfoCallback();

	creos_messages::CoordinateSystemInfo GetLatestCoordinateSystemInfo();

private:
	std::mutex                           coordinate_system_mutex_;
	creos_messages::CoordinateSystemInfo latest_coordinate_system_info_;
	void coordinateSystemInfoCallback(const creos_messages::CoordinateSystemInfo &msg);
};
