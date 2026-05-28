// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <queue>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cstdint>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <tf2_ros/transform_broadcaster.h>

namespace rm_serial_driver
{

    class RMSerialDriver : public rclcpp::Node
    {
    public:
        explicit RMSerialDriver(const std::string &node_name, const rclcpp::NodeOptions &options);

        ~RMSerialDriver() override;

    protected:
        // ========== subclass must implement ==========
        //
        // Read & parse one packet from the serial port, then publish.
        // Called in a loop by receiveData().
        virtual void processOnePacket() = 0;

        // ========== subclass calls to send ==========
        void sendRaw(const std::vector<uint8_t> &data);

        // ========== shared serial helpers ==========
        void getParams();
        void reopenPort();
        void startReceive();

        // Serial port — subclass may use directly in processOnePacket()
        std::unique_ptr<IoContext> owned_ctx_;
        std::string device_name_;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    private:
        void receiveData();
        std::thread receive_thread_;
    };

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
