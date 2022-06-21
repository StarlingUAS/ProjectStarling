#ifndef ESTOP_HPP
#define ESTOP_HPP

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
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <cmath>

#include <std_msgs/msg/empty.hpp>
#include "mavros_msgs/srv/command_long.hpp"


class EstopHandler : public rclcpp::Node
{
    public:
        EstopHandler();

    private:
        void emergency_stop();
        rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr        mavros_command_srv;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr               estop_sub;
};


#endif
