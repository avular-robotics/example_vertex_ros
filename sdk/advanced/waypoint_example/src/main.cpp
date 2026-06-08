// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Example of how to use the CreOS SDK to automatically take off and fly
 * a waypoint mission. This example uses the Job API to define a Job.
 ****************************************************************************/
#include <CLI11.hpp>
#include <iostream>
#include <optional>

#include <creos/client.hpp>
#include <creos/autopilot/navigation.hpp>
#include <creos/job/job.hpp>
#include <creos/job/wait.hpp>
#include <creos/job/waypoint.hpp>
#include <creos/messages/controller_state.hpp>
#include <creos/messages/pose.hpp>

#include <common/logging.hpp>
#include <common/drone_state.hpp>
#include <common/remote_controller_interface.hpp>
#include <common/flight_controller.hpp>

enum class JobState
{
    kStartJob,
    kJobRunning,
};

void spin(unsigned update_frequency_hz)
{
    usleep(1000000 / update_frequency_hz);
}

creos::Client createClient(const std::string_view host)
{
    if(host.empty())
    {
        return creos::Client();
    }
    return creos::Client(host);
}

creos::Job createJob1(creos::Client &client, const std::array<float, 3> current_position)
{
    // Create a new job with a unique identifier
    creos::Job job(client, "example-job-001");

    //! [job_creation]
    // Set maximum speed for the entire job. This can be overridden for individual waypoints.
    // If a max speed is not set a value of 1 m/s is used by default.
    job.setMaxSpeed(4.0);

    //! [takeoff_task]
    // Add a takeoff task at the beginning of the mission.
    // Take of height will be the configured takeoff height in the settings.
    job.addTakeOff(creos::TakeOff());
    //! [takeoff_task]

    //! [waypoint_basic]
    // Add a waypoint with custom navigation settings using fluent interface
    // Coordinates are in the robot's local coordinate frame (meters)
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0], current_position[1] + 2.0,
                                                   current_position[2] + 10.0))
            .setHeading(90.0)                                        // Face east (90 degrees)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)); // Heading relative to north

    //! [waypoint_basic]
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0], current_position[1] + 10.0,
                                                   current_position[2] + 10.0))
            .setHeading(0.0)                                       // Face north (0 degrees)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth) // Heading relative to north
            .setMaxSpeed(1.0));                                    // Slow down for this waypoint

    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0], current_position[1] + 10.0,
                                                   current_position[2] + 14.0))
            .setHeading(-90.0)                                       // Face west (-90 degrees)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)); // Heading relative to north

    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0], current_position[1] + 2.0,
                                                   current_position[2] + 14.0))
            .setHeading(0.0)                                         // Face north (0 degrees)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)); // Heading relative to north

    //! [waypoint_return]
    // Add a final waypoint to return to start
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0], current_position[1] + 2.0,
                                                   current_position[2] + 10.0))
            .setHeading(180.0)                                       // Face south (180 degrees)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)); // Heading relative to north
    //! [waypoint_return]

    //! [land_task]
    // Add a land task to finish the mission.
    job.addLand(creos::Land());
    //! [land_task]

    // Print job description before starting
    spdlog::info("{}", job.describe());

    return job;
}

void reportJobStatus(creos_messages::JobStatus status, size_t &finished_tasks, int &waypoint_index)
{
    if(finished_tasks < status.tasks.size())
    {
        const auto task = status.tasks[finished_tasks];

        // Log task completion
        if(task.status == creos_messages::ExecutionStatus::kSucceeded)
        {
            switch(task.type)
            {
            case creos_messages::TaskType::kTakeOff:
                spdlog::info("Takeoff completed");
                break;
            case creos_messages::TaskType::kMoveTroughPoses:
                waypoint_index++;
                spdlog::info("Waypoint {} reached", waypoint_index);
                break;
            case creos_messages::TaskType::kLand:
                spdlog::info("Landing completed");
                break;
            default:
                break;
            }
            finished_tasks++;
        }
    }
}

int main(int argc, char **argv)
{
    CLI::App app{"Waypoint Example - Showcase how to use the CreOS SDK to automatically take off "
                 "and fly through a series of waypoints, and then land. The example uses the Job "
                 "API to define a Job consisting of waypoint tasks with custom heading and speed "
                 "settings."};

    unsigned       update_frequency_hz = 100;
    ControllerType controller_type     = ControllerType::kHerelink;
    std::string    host                = "";
    unsigned       job_number          = 1;

    app.add_flag(
        "-v,--verbose", [](auto) { spdlog::set_level(spdlog::level::debug); },
        "Enable verbose output");
    app.add_option("--host", host,
                   "<hostname:port> of the CREOS agent. When omitted, the 'CREOS_HOST:CREOS_PORT' "
                   "environment variables are used or localhost:7200 is used as a fallback.");
    app.add_option("-f,--frequency", update_frequency_hz,
                   "Update frequency in Hz. Default is 100 Hz.");
    app.add_option("--job", job_number, "Select which job to run.");
    app.add_flag_callback(
        "--jeti", [&controller_type]() { controller_type = ControllerType::kJeti; },
        "Use Jeti controller instead of Herelink controller");
    CLI11_PARSE(app, argc, argv);

    setup_logging("waypoint_example");

    // Connect to the CreOS server
    creos::Client client = createClient(host);

    // Setup DroneState
    std::shared_ptr<DroneState> drone_state = std::make_shared<DroneState>();
    client.sensors()->subscribeToPose(drone_state->GetGlobalPoseCallback());
    client.setpoint_control()->subscribeToCurrentControlSource(
        drone_state->GetControlSourceCallback());
    client.diagnostics()->subscribeToState(drone_state->GetStateCallback());

    // Setup Controller input
    std::shared_ptr<IRemoteController> controller = CreateRemoteController(controller_type);
    client.sensors()->subscribeToRemoteController(controller->GetControllerStateCallback());

    bool execution_active = false;
    controller->RegisterActivationButtonCallback(
        [&execution_active]()
        {
            execution_active = !execution_active;
            spdlog::info("Execution active: {}", execution_active ? "true" : "false");
        });

    JobState job_state = JobState::kStartJob;

    // Function for create job
    std::function<creos::Job(creos::Client &, const std::array<float, 3>)> createJobFunc;
    switch(job_number)
    {
    case 1:
        createJobFunc = createJob1;
        break;
    default:
        spdlog::error("Invalid job number: {}. Valid job numbers are 1.", job_number);
        return 1;
    }

    spdlog::info("Selected job number: {}. Press the activation button on the controller to start "
                 "execution.",
                 job_number);

    size_t                      finished_tasks = 0;
    int                         waypoint_index = 0;
    creos_messages::JobStatus   status = {creos_messages::ExecutionStatus::kNotLoaded, {}, {}};
    std::shared_ptr<creos::Job> job    = nullptr;
    while((status.status != creos_messages::ExecutionStatus::kSucceeded) &&
          (status.status != creos_messages::ExecutionStatus::kCanceled) &&
          (status.status != creos_messages::ExecutionStatus::kFailed))
    {
        if(execution_active)
        {
            switch(job_state)
            {
            case JobState::kStartJob:
            {
                // Create the job when execution is activated. We create a new job each time
                // execution is activated, so if you deactivate and reactivate execution, a new job
                // will be created and started.
                job =
                    std::make_shared<creos::Job>(createJobFunc(client, drone_state->GetPosition()));

                // Start the job execution
                job->start();
                job_state = JobState::kJobRunning;
                break;
            }
            case JobState::kJobRunning:
            {
                status = job->getJobStatus();
                reportJobStatus(status, finished_tasks, waypoint_index);
                break;
            }
            }
        }
        else
        {
            if(job_state == JobState::kJobRunning)
            {
                job->stop();
                break;
            }
        }
        spin(update_frequency_hz);
    }
    if(status.status == creos_messages::ExecutionStatus::kSucceeded)
    {
        std::cout << std::endl;
        std::cout << "Example completed successfully!" << std::endl;
    }
    else
    {
        std::cout << std::endl;
        std::cout << "Example did not complete successfully." << std::endl;
    }

    return 0;
}
