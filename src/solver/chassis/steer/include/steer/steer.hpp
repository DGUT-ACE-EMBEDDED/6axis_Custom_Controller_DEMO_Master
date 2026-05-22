#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/master_chassis_cmd.hpp"
#include "rm_interfaces/msg/chassis_solve.hpp"
#include "rm_interfaces/msg/chassis_state.hpp"

namespace solver
{
    namespace chassis
    {
        class Steer:public rclcpp::Node
        {
            public:
            
            explicit Steer(const rclcpp::NodeOptions &options);
            ~Steer() override;

            private:

            void solverCallback();

            void getSetValueCallback(const rm_interfaces::msg::MasterChassisCmd::SharedPtr msg);
            
            void getSteerAngleCallback(const rm_interfaces::msg::ChassisState::SharedPtr msg);

            void getParams();

            struct 
            {
             float x_set;
             float y_set;
             float yaw_set;
            }input;

            struct 
            {
                struct{
                float rd;
                float rf;
                float ld;
                float lf;
                }wheel;
                struct 
                {
                    float rd;
                    float rf;
                    float ld;
                    float lf;
                }steer;
            }output;

            struct
            {
               uint16_t lf;
               uint16_t ld;
               uint16_t rf;
               uint16_t rd;
            }init_steer_encode;
            
            struct
            {
                float lf;
                float ld;
                float rf;
                float rd;
            }steer_now_angle;
            

            float car_length;
            float car_width;
            float rotate_gain;
            
            rclcpp::Subscription<rm_interfaces::msg::MasterChassisCmd>::SharedPtr cmd_sub;
            rclcpp::Publisher<rm_interfaces::msg::ChassisSolve>::SharedPtr solve_value_pub;
            rclcpp::Subscription<rm_interfaces::msg::ChassisState>::SharedPtr steer_angle_sub;
            rclcpp::TimerBase::SharedPtr timer;
        };
    }
}