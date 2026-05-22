// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.
#include "rm_serial_driver/rm_serial_driver.hpp"

int i = 0;


namespace rm_serial_driver
{
    
    RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions &options) :
        Node("rm_serial_driver", options), owned_ctx_{new IoContext(2)}, serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
    {
        RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");
        // is_hero = this->declare_parameter("is_hero", false);
        getParams();
        // TF broadcaster
        timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // Create Publisher
        latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);

        // Detect parameter client
        detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "detector_node");
        tracker_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "tracker_node");

        // Tracker reset service client
        reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

        try
        {
            serial_driver_->init_port(device_name_, *device_config_);
            if (!serial_driver_->port()->is_open())
            {
                serial_driver_->port()->open();
                receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
            throw ex;
        }

        // Create Subscription
        // 在订阅者
        
    }

    RMSerialDriver::~RMSerialDriver()
    {
        if (receive_thread_.joinable())
        {
            receive_thread_.join();
        }

        if (serial_driver_->port()->is_open())
        {
            serial_driver_->port()->close();
        }

        if (owned_ctx_)
        {
            owned_ctx_->waitForExit();
        }
    }

    void RMSerialDriver::receiveData()
    {
        std::vector<uint8_t> header(1);
        std::vector<uint8_t> data;
        data.reserve(sizeof(ReceivePacket));

        while (rclcpp::ok())
        {
            try
            {
                serial_driver_->port()->receive(header);
                if (header[0] == 0xEE)
                {
                    data.resize(sizeof(ReceivePacket) - 1);
                    serial_driver_->port()->receive(data);
                    data.insert(data.begin(), header [0]);
                    ReceivePacket packet = fromVector(data);
                    if(packet.header == 0xEE && packet.checksum == 0xED)
                    {
                        i++;
                        RCLCPP_INFO(get_logger(), "%d", i);
                    }
                    else
                    {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid packet received: header=%02X, checksum=%02X", packet.header, packet.checksum);
                    }
                  
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
                }
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
                reopenPort();
            }
        }
    }


    //如果要发送

    // void RMSerialDriver::sendData(const auto_aim_interfaces::msg::JointInfoSet::SharedPtr msg)
    // {
    //     try
    //     {
    //         SendPacket packet;
            

    //         serial_driver_->port()->send(data);
    //     }
    //     catch (const std::exception &ex)
    //     {
    //         RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    //         reopenPort();
    //     }
    // }

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
            {
                fc = FlowControl::NONE;
            }
            else if (fc_string == "hardware")
            {
                fc = FlowControl::HARDWARE;
            }
            else if (fc_string == "software")
            {
                fc = FlowControl::SOFTWARE;
            }
            else
            {
                throw std::invalid_argument{"The flow_control parameter must be one of: none, software, or hardware."};
            }
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
            {
                pt = Parity::NONE;
            }
            else if (pt_string == "odd")
            {
                pt = Parity::ODD;
            }
            else if (pt_string == "even")
            {
                pt = Parity::EVEN;
            }
            else
            {
                throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
            }
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
            {
                sb = StopBits::ONE;
            }
            else if (sb_string == "1.5")
            {
                sb = StopBits::ONE_POINT_FIVE;
            }
            else if (sb_string == "2" || sb_string == "2.0")
            {
                sb = StopBits::TWO;
            }
            else
            {
                throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
            }
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
            throw ex;
        }

        device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
    }

    void RMSerialDriver::reopenPort()
    {
        RCLCPP_WARN(get_logger(), "Attempting to reopen port");
        try
        {
            if (serial_driver_->port()->is_open())
            {
                serial_driver_->port()->close();
            }
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

    void RMSerialDriver::setParam(const rclcpp::Parameter &param)
    {
        if (!detector_param_client_->service_is_ready())
        {
            // RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
            return;
        }

        if (!set_param_future_.valid() || set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            // RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
            set_param_future_ =
                detector_param_client_->set_parameters({param},
                                                       [this, param](const ResultFuturePtr &results)
                                                       {
                                                           for (const auto &result : results.get())
                                                           {
                                                               if (!result.successful)
                                                               {
                                                                   RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                                                                   return;
                                                               }
                                                           }
                                                           //    RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
                                                           initial_set_param_ = true;
                                                       });
        }
    }

    // void RMSerialDriver::setJointParam(const rclcpp::Parameter &param)
    // {
    //     if (!tracker_param_client_->service_is_ready())
    //     {
    //         // RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    //         return;
    //     }

    //     if (!set_param_future_.valid() || set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    //     {
    //         // RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    //         set_param_future_ =
    //             tracker_param_client_->set_parameters({param},
    //                                                   [this, param](const ResultFuturePtr &results)
    //                                                   {
    //                                                       for (const auto &result : results.get())
    //                                                       {
    //                                                           if (!result.successful)
    //                                                           {
    //                                                               RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
    //                                                               return;
    //                                                           }
    //                                                       }
    //                                                       //   RCLCPP_INFO(get_logger(), "Successfully set joint_value to %lf!",
    //                                                       param.as_double());
    //                                                   });
    //     }
    // }



    bool RMSerialDriver::isReadyForNewRequest() const
    {
        return !set_param_future_.valid() || set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }

    void RMSerialDriver::sendParameterRequest(const rclcpp::Parameter &param)
    {
        // RCLCPP_INFO(get_logger(), "Sending param: %s=%.2f", param.get_name().c_str(), param.as_double());

        set_param_future_ =
            tracker_param_client_->set_parameters({param}, [this](const ResultFuturePtr &results) { handleParameterResponse(results); });
    }

    void RMSerialDriver::handleParameterResponse(const ResultFuturePtr &results)
    {
        try
        {
            auto result = results.get().front();
            if (!result.successful)
            {
                RCLCPP_ERROR(get_logger(), "Failed to set param: %s", result.reason.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Param set exception: %s", e.what());
        }

        std::lock_guard<std::mutex> lock(param_mutex_);
        if (!param_queue_.empty())
        {
            processParamQueue();
        }
    }

    void RMSerialDriver::processParamQueue()
    {
        while (!param_queue_.empty() && isReadyForNewRequest())
        {
            auto next_param = param_queue_.front();
            param_queue_.pop();
            sendParameterRequest(next_param);
        }
    }


} // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
