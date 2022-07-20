#ifndef PING_HPP
#define PING_HPP

/*
 * Copyright (C) 2022 University of Bristol
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
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <cmath>

#include <std_msgs/msg/header.hpp>

class PingHandler : public rclcpp::Node
{
    public:
        PingHandler();
    
    private:
        void handle_ping(const std_msgs::msg::Header::SharedPtr s);

        rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr ping_sub;
        rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ping_pub;

        std::string vehicle_id;
};


#endif