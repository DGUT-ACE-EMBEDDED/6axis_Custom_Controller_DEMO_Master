// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{

    RMSerialDriver::RMSerialDriver(const std::string &node_name, const rclcpp::NodeOptions &options)
        : Node(node_name, options), owned_ctx_{new IoContext(2)},
          serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
    {
        RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");
        getParams();

        try
        {
            serial_driver_->init_port(device_name_, *device_config_);
            if (!serial_driver_->port()->is_open())
            {
                serial_driver_->port()->open();
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s",
                         device_name_.c_str(), ex.what());
            throw ex;
        }
    }

    RMSerialDriver::~RMSerialDriver()
    {
        if (receive_thread_.joinable())
            receive_thread_.join();

        if (serial_driver_->port()->is_open())
            serial_driver_->port()->close();

        if (owned_ctx_)
            owned_ctx_->waitForExit();
    }

    void RMSerialDriver::receiveData()
    {
        while (rclcpp::ok())
        {
            try
            {
                processOnePacket();
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20,
                                      "Error while receiving data: %s", ex.what());
                reopenPort();
            }
        }
    }

    void RMSerialDriver::sendRaw(const std::vector<uint8_t> &data)
    {
        try
        {
            serial_driver_->port()->send(data);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
            reopenPort();
        }
    }

    void RMSerialDriver::startReceive()
    {
        receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }

    void RMSerialDriver::getParams()
    {
        using FlowControl = drivers::serial_driver::FlowControl;
        using Parity = drivers::serial_driver::Parity;
        using StopBits = drivers::serial_driver::StopBits;

        uint32_t baud_rate{};
        auto fc = FlowControl::NONE;
        auto pt = Parity::NONE;
        auto sb = StopBits::ONE;

        try
        {
            device_name_ = declare_parameter<std::string>("device_name", "");
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
            throw ex;
        }

        try
        {
            baud_rate = declare_parameter<int>("baud_rate", 0);
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
            throw ex;
        }

        try
        {
            const auto fc_string = declare_parameter<std::string>("flow_control", "none");
            if (fc_string == "none")
                fc = FlowControl::NONE;
            else if (fc_string == "hardware")
                fc = FlowControl::HARDWARE;
            else if (fc_string == "software")
                fc = FlowControl::SOFTWARE;
            else
                throw std::invalid_argument{
                    "The flow_control parameter must be one of: none, software, or hardware."};
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
            throw ex;
        }

        try
        {
            const auto pt_string = declare_parameter<std::string>("parity", "");
            if (pt_string == "none")
                pt = Parity::NONE;
            else if (pt_string == "odd")
                pt = Parity::ODD;
            else if (pt_string == "even")
                pt = Parity::EVEN;
            else
                throw std::invalid_argument{
                    "The parity parameter must be one of: none, odd, or even."};
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
            throw ex;
        }

        try
        {
            const auto sb_string = declare_parameter<std::string>("stop_bits", "");
            if (sb_string == "1" || sb_string == "1.0")
                sb = StopBits::ONE;
            else if (sb_string == "1.5")
                sb = StopBits::ONE_POINT_FIVE;
            else if (sb_string == "2" || sb_string == "2.0")
                sb = StopBits::TWO;
            else
                throw std::invalid_argument{
                    "The stop_bits parameter must be one of: 1, 1.5, or 2."};
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
            throw ex;
        }

        device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
            baud_rate, fc, pt, sb);
    }

    void RMSerialDriver::reopenPort()
    {
        RCLCPP_WARN(get_logger(), "Attempting to reopen port");
        try
        {
            if (serial_driver_->port()->is_open())
                serial_driver_->port()->close();
            serial_driver_->port()->open();
            RCLCPP_INFO(get_logger(), "Successfully reopened port");
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
            if (rclcpp::ok())
            {
                rclcpp::sleep_for(std::chrono::seconds(1));
                reopenPort();
            }
        }
    }

} // namespace rm_serial_driver
