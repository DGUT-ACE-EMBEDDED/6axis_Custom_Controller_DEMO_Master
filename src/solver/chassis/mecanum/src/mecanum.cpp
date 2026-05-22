#include "mecanum/mecanum.hpp"

namespace solver
{
    namespace chassis
    {
        Mecanum::Mecanum(const rclcpp::NodeOptions &options):Node("mecanum_solver",options)
        {
            RCLCPP_INFO(this->get_logger(),"this is Mecanum");
            getParams();

            rotate_gain = (car_length +car_width)/2;
            
            solve_value_pub = this->create_publisher<rm_interfaces::msg::ChassisSolve>("/solver/mecanum",10);
            cmd_sub = this->create_subscription<rm_interfaces::msg::MasterChassisCmd>("/master/chassis/set",10,std::bind(&Mecanum::solverCallback,this,std::placeholders::_1));
        }
        Mecanum::~Mecanum()
        {
        }

        void Mecanum::solverCallback(const rm_interfaces::msg::MasterChassisCmd::SharedPtr msg)
        {
            auto solve_value = rm_interfaces::msg::ChassisSolve();

            input.x_set = msg->x_set;
            input.y_set = msg->y_set;
            input.yaw_set = msg->yaw_set;
            
            solve_value.lf = output.lf = input.x_set - input.y_set - input.yaw_set *rotate_gain;
            solve_value.rf = output.rf = input.x_set + input.y_set + input.yaw_set *rotate_gain;
            solve_value.ld = output.ld = input.x_set + input.y_set - input.yaw_set *rotate_gain;
            solve_value.rd = output.rd = input.x_set - input.y_set + input.yaw_set *rotate_gain;
            
            solve_value_pub->publish(solve_value);
        }

        void Mecanum::getParams()
        {
            try
            {
               car_length = declare_parameter<float>("car_length",0.0f);
            }
            catch (rclcpp::ParameterTypeException &ex)
            {
                RCLCPP_ERROR(get_logger(), "The car_length provided was invalid");
                throw ex;
            }

            try
            {
               car_width = declare_parameter<float>("car_width",0.0f);
            }
            catch (rclcpp::ParameterTypeException &ex)
            {
                RCLCPP_ERROR(get_logger(), "The car_width provided was invalid");
                throw ex;
            }
            RCLCPP_INFO(this->get_logger(),"%f,%f",car_length,car_width);
        }
    }
}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(solver::chassis::Mecanum)