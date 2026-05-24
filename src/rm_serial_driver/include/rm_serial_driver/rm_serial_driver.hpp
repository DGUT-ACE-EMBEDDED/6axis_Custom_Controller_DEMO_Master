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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
// C++ system
#include <cstdint>
#include <functional>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// tf2
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>



#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"


namespace rm_serial_driver
{
    class RMSerialDriver : public rclcpp::Node
    {
    public:
        explicit RMSerialDriver(const rclcpp::NodeOptions &options);

        ~RMSerialDriver() override;

    private:
        void getParams();

        void receiveData();

      

        void reopenPort();

        void setParam(const rclcpp::Parameter &param);
        

        void resetTracker();

        // Serial port
        std::unique_ptr<IoContext> owned_ctx_;
        std::string device_name_;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
        // Param client to set detect_colr
        using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
        bool initial_set_param_ = false;
        uint8_t previous_receive_color_ = 0;
        rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
        rclcpp::AsyncParametersClient::SharedPtr tracker_param_client_;

        ResultFuturePtr set_param_future_;

        // Service client to reset tracker
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;
        // Broadcast tf from odom to gimbal_link
        double timestamp_offset_ = 0;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        //rclcpp::Subscription<auto_aim_interfaces::msg::JointInfoSet>::SharedPtr target_sub_;
        // For debug usage
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
        std::thread receive_thread_;


        bool isReadyForNewRequest() const;

        void sendParameterRequest(const rclcpp::Parameter &param);

        void handleParameterResponse(const ResultFuturePtr &results);

        void processParamQueue();
        std::mutex param_mutex_;
        std::queue<rclcpp::Parameter> param_queue_;
    };

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
