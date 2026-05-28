// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__DEVICE_B_DRIVER_HPP_
#define RM_SERIAL_DRIVER__DEVICE_B_DRIVER_HPP_

#include "rm_serial_driver/rm_serial_driver.hpp"

#include "rm_serial_driver/packet.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <queue>
#include <mutex>

namespace rm_serial_driver
{

class DeviceBDriver : public RMSerialDriver
{
public:
    explicit DeviceBDriver(const rclcpp::NodeOptions &options);

private:
    void processOnePacket() override;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void setParam(const rclcpp::Parameter &param);
    void resetTracker();

    void sendParameterRequest(const rclcpp::Parameter &param);
    void handleParameterResponse(
        const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> &results);
    void processParamQueue();
    bool isReadyForNewRequest() const;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

    
    double timestamp_offset_ = 0;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  
    rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
    rclcpp::AsyncParametersClient::SharedPtr tracker_param_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

    using ResultFuturePtr =
        std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
    ResultFuturePtr set_param_future_;
    bool initial_set_param_ = false;
    uint8_t previous_receive_color_ = 0;

    std::mutex param_mutex_;
    std::queue<rclcpp::Parameter> param_queue_;

};

}  

#endif  // RM_SERIAL_DRIVER__DEVICE_B_DRIVER_HPP_
