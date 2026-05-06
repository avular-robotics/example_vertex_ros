#include "coordinate_manager.h"

std::function<void(const creos_messages::CoordinateSystemInfo &)> CoordinateManager::
	GetCoordinateSystemInfoCallback()
{
	return std::bind(&CoordinateManager::coordinateSystemInfoCallback, this, std::placeholders::_1);
}

creos_messages::CoordinateSystemInfo CoordinateManager::GetLatestCoordinateSystemInfo()
{
	std::lock_guard<std::mutex> lock(coordinate_system_mutex_);
	return latest_coordinate_system_info_;
}

void CoordinateManager::coordinateSystemInfoCallback(const creos_messages::CoordinateSystemInfo &msg)
{
	std::lock_guard<std::mutex> lock(coordinate_system_mutex_);
	latest_coordinate_system_info_ = msg;
}
