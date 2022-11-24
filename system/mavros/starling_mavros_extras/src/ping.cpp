/*
 * Copyright (C) 2021 University of Bristol
 *
 * Author: Mickey Li <mickey.li@bristol.ac.uk> (University of Bristol)
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#include "ping.hpp"

PingHandler::PingHandler() :
	Node("ping_handler")
{
    this->declare_parameter("vehicle_id");
    rclcpp::Parameter id_param = this->get_parameter("vehicle_id");
    this->vehicle_id = id_param.as_string();

    this->ping_sub = this->create_subscription<std_msgs::msg::Header>(
        "/ping_source", 1, 
        [this](const std_msgs::msg::Header::SharedPtr s){
            this->handle_ping(s);
        }
    );

    this->ping_pub = this->create_publisher<std_msgs::msg::Header>("/ping_sink", 10);

    RCLCPP_INFO(this->get_logger(), "Ping Initialised");
}

void PingHandler::handle_ping(const std_msgs::msg::Header::SharedPtr s) {
    std_msgs::msg::Header msg;
    msg.frame_id = this->vehicle_id;
    msg.stamp = s->stamp;
    this->ping_pub->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto handler = std::make_shared<PingHandler>();
    rclcpp::spin(handler);
    rclcpp::shutdown();
    return 0;
}