// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "rm_serial_driver/device_a_driver.hpp"

namespace rm_serial_driver
{

    DeviceADriver::DeviceADriver(const rclcpp::NodeOptions &options)
        : RMSerialDriver("device_a_driver", options)
    {
        RCLCPP_INFO(get_logger(), "Start DeviceADriver!");

        // TF broadcaster
        timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Publishers
        latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
        target_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>("/target/pose", 10);

        // Parameter clients
        detector_param_client_ =
            std::make_shared<rclcpp::AsyncParametersClient>(this, "detector_node");
        tracker_param_client_ =
            std::make_shared<rclcpp::AsyncParametersClient>(this, "tracker_node");

        // Service client
        reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

        startReceive();
    }

    void DeviceADriver::processOnePacket()
    {
        std::vector<uint8_t> header(1);
        std::vector<uint8_t> data;
        data.reserve(sizeof(ReceivePacket));

        serial_driver_->port()->receive(header);

        if (header[0] != 0xEE)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20,
                                 "Invalid header: %02X", header[0]);
            return;
        }

        data.push_back(header[0]);
        std::vector<uint8_t> body(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(body);
        data.insert(data.end(), body.begin(), body.end());

        ReceivePacket packet = fromVector(data);

        if (packet.checksum != 0xED)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20,
                                 "checksum mismatch: header=%02X", packet.header);
            return;
        }

        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "end";
        msg.pose.position.x = packet.x ;
        msg.pose.position.y = packet.y ;
        msg.pose.position.z = packet.z ;
        msg.pose.orientation.x = packet.qx;
        msg.pose.orientation.y = packet.qy;
        msg.pose.orientation.z = packet.qz;
        msg.pose.orientation.w = packet.qw;
        target_pose_pub_->publish(msg);
    }

  

    void DeviceADriver::setParam(const rclcpp::Parameter &param)
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

    void DeviceADriver::resetTracker()
    {
        if (!reset_tracker_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(get_logger(), "Tracker reset service not available");
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        reset_tracker_client_->async_send_request(request);
    }

    bool DeviceADriver::isReadyForNewRequest() const
    {
        return !set_param_future_.valid() ||
               set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }

    void DeviceADriver::sendParameterRequest(const rclcpp::Parameter &param)
    {
        set_param_future_ = tracker_param_client_->set_parameters(
            {param}, [this](const ResultFuturePtr &results)
            { handleParameterResponse(results); });
    }

    void DeviceADriver::handleParameterResponse(const ResultFuturePtr &results)
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

    void DeviceADriver::processParamQueue()
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
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::DeviceADriver)
