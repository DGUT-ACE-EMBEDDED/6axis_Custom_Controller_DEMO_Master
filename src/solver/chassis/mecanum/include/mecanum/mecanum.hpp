#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/master_chassis_cmd.hpp"
#include "rm_interfaces/msg/chassis_solve.hpp"



namespace solver
{
    namespace chassis
    {
        class Mecanum : public rclcpp::Node
        {
        public:
            
            explicit Mecanum(const rclcpp::NodeOptions &options);
            ~Mecanum() override;

        private:
           
            void solverCallback(const rm_interfaces::msg::MasterChassisCmd::SharedPtr msg);

            void getParams();

            struct InputSpeedValue
            {
             float x_set;
             float y_set;
             float yaw_set;
            };

            struct OutputSpeedValue
            {
                float rd;
                float rf;
                float ld;
                float lf;
            };

            float car_length;
            float car_width;
            float rotate_gain;

            
            rclcpp::Subscription<rm_interfaces::msg::MasterChassisCmd>::SharedPtr cmd_sub;
            rclcpp::Publisher<rm_interfaces::msg::ChassisSolve>::SharedPtr solve_value_pub;

            InputSpeedValue input = {};
            OutputSpeedValue output = {};
        };
    }

}