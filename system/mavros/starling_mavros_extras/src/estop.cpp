/*
 * Trajectory follower
 * Copyright (C) 2021 University of Bristol
 *
 * Author: Mickey Li <mickey.li@bristol.ac.uk> (University of Bristol)
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#include "estop.hpp"

EstopHandler::EstopHandler() :
	Node("estop_handler")
{
    this->mavros_command_srv = this->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command", rmw_qos_profile_services_default);
    this->estop_sub = this->create_subscription<std_msgs::msg::Empty>(
        "/emergency_stop", 1, 
        [this](const std_msgs::msg::Empty::SharedPtr s){
            (void)s;
            this->emergency_stop();
        }
    );

    RCLCPP_INFO(this->get_logger(), "ESTOP Initialised");
}

void EstopHandler::emergency_stop() {
    RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP RECIEVED, SENDING KILL MESSAGE TO AUTOPILOT");
    // Kill Drone Rotors
    auto commandCall = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    commandCall->broadcast = false;
    commandCall->command = 400;
    commandCall->param2 = 21196.0;
    while (!this->mavros_command_srv->wait_for_service(std::chrono::duration<float>(0.1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for command service. Exiting.");
            throw std::runtime_error("Interrupted while waiting for command service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto result_future = this->mavros_command_srv->async_send_request(commandCall);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto handler = std::make_shared<EstopHandler>();
    rclcpp::spin(handler);
    rclcpp::shutdown();
    return 0;
}