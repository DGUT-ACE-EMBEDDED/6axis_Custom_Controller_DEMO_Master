// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "rm_serial_driver/device_b_driver.hpp"

#include <cmath>

namespace rm_serial_driver
{

    DeviceBDriver::DeviceBDriver(const rclcpp::NodeOptions &options)
        : RMSerialDriver("device_b_driver", options)
    {
        RCLCPP_INFO(get_logger(), "Start DeviceBDriver!");

        // TF broadcaster
        timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Publishers
        latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&DeviceBDriver::jointStateCallback, this, std::placeholders::_1));

        // Parameter clients
        detector_param_client_ =
            std::make_shared<rclcpp::AsyncParametersClient>(this, "detector_node");
        tracker_param_client_ =
            std::make_shared<rclcpp::AsyncParametersClient>(this, "tracker_node");

        // Service client
        reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

        startReceive();
    }

    void DeviceBDriver::processOnePacket()
    {
    }

    void DeviceBDriver::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        SendPacket packet;
        packet.header = 0xEE;
        packet.joint1 = msg->position[0];
        packet.joint2 = msg->position[1];
        packet.joint3 = msg->position[2];
        packet.joint4 = msg->position[3];
        packet.joint5 = msg->position[4];
        packet.joint6 = msg->position[5];
        packet.checksum = 0xED;
        auto data = toVector(packet);
        sendRaw(data);
    }

    void DeviceBDriver::setParam(const rclcpp::Parameter &param)
    {
        if (!detector_param_client_->service_is_ready())
            return;

        if (!set_param_future_.valid() ||
            set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            set_param_future_ = detector_param_client_->set_parameters(
                {param},
                [this, param](const ResultFuturePtr &results)
                {
                    for (const auto &result : results.get())
                    {
                        if (!result.successful)
                        {
                            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s",
                                         result.reason.c_str());
                            return;
                        }
                    }
                    initial_set_param_ = true;
                });
        }
    }

    void DeviceBDriver::resetTracker()
    {
        if (!reset_tracker_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(get_logger(), "Tracker reset service not available");
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        reset_tracker_client_->async_send_request(request);
    }

    bool DeviceBDriver::isReadyForNewRequest() const
    {
        return !set_param_future_.valid() ||
               set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }

    void DeviceBDriver::sendParameterRequest(const rclcpp::Parameter &param)
    {
        set_param_future_ = tracker_param_client_->set_parameters(
            {param}, [this](const ResultFuturePtr &results)
            { handleParameterResponse(results); });
    }

    void DeviceBDriver::handleParameterResponse(const ResultFuturePtr &results)
    {
        try
        {
            auto result = results.get().front();
            if (!result.successful)
                RCLCPP_ERROR(get_logger(), "Failed to set param: %s", result.reason.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Param set exception: %s", e.what());
        }

        std::lock_guard<std::mutex> lock(param_mutex_);
        if (!param_queue_.empty())
            processParamQueue();
    }

    void DeviceBDriver::processParamQueue()
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
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::DeviceBDriver)
