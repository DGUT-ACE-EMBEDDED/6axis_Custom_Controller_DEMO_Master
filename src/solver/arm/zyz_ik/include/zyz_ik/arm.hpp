#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace solver
{
    namespace arm
    {
        class ZYZIK : public rclcpp::Node
        {
        public:
            ZYZIK(const rclcpp::NodeOptions &options);
            ~ZYZIK() override;

        private:
            void update();
            void getParams();
            void computeIK();
            void computeFK(const double j[6], double &x, double &y, double &z);
            void j2LimitCalculate();
            


            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
            rclcpp::TimerBase::SharedPtr timer_;
            void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

            struct JointLimit { double lower, upper; };
            struct { JointLimit joint_1, joint_2, joint_3, joint_4, joint_5, joint_6; } joint_limits_;
            struct { double l2, l3, deta_a, deta_d; } dh_params_;
            struct JointAngles { double joint_1, joint_2, joint_3, joint_4, joint_5, joint_6; };
            JointAngles current_joint_angles_{};
            JointAngles target_joint_angles_{};
            JointAngles start_joint_angles_{};

            rclcpp::Time transition_start_time_{0, 0, RCL_ROS_TIME};
            double transition_duration_;

            struct
            {
                double x, y, z;
                double roll, pitch, yaw;
                bool updated = false;
            } target_pose_;

            double l4d4_across_, angle_l4d4_;
            Eigen::Matrix4d T6t_;
            Eigen::Matrix4d Tend26_inv_;
            Eigen::Matrix3d target_R;
            float EQS_VAL = 0.01 ;
            float tool_link = 0;
            float joint3_zero;
            double max_length;
            double max_angle;
            
        };
    }
}