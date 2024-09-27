// Copyright 2024 Avular B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include <rclcpp/rclcpp.hpp>

#include "circle_example_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CircleExampleNode>(rclcpp::NodeOptions());
    rclcpp::Rate loop_rate(std::chrono::milliseconds(CircleExampleNode::kMsInSecond / CircleExampleNode::kUpdateFrequency_Hz));

    RCLCPP_INFO(
        node->get_logger(),
        "\n---------------------------------------------------------------------------"
        "----\n-----------------------------------------------------------------------"
        "--------\n------- When testing for the first time, always remove the "
        "propellers!! -------\n---------------------------------------------------------------"
        "----------------\n-----------------------------------------------------------"
        "--------------------\n");
    RCLCPP_INFO(
        node->get_logger(),
        "\n---------------------------------------------------------------------------------------"
        "\n- First, change "
        "the control mode of the drone to 'Auto'. Start "
        "the script by pressing -\n- the 'C' button. Based on the current "
        "state of the drone, it will arm, take-off and  -\n- start flying. For more "
        "information, please read the "
        "README.\t\t\t      "
        "-\n------------------------------------------------------------------------"
        "--"
        "-"
        "--------"
        "-"
        "-"
        "--");

    while(rclcpp::ok())
    {
        node->ExecuteExample();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
