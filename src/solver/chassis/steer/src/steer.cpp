#include "steer/steer.hpp"

namespace solver
{
    namespace chassis
    {
        //这里特指4wis
        Steer::Steer(const rclcpp::NodeOptions &options):Node("steer_solver",options)
        {
            RCLCPP_INFO(this->get_logger(),"this is Steer");
            getParams();

            solve_value_pub = this->create_publisher<rm_interfaces::msg::ChassisSolve>("/solver/steer",10);
            cmd_sub = this->create_subscription<rm_interfaces::msg::MasterChassisCmd>("/master/chassis/set",10,std::bind(&Steer::getSetValueCallback,this,std::placeholders::_1));
            steer_angle_sub = this->create_subscription<rm_interfaces::msg::ChassisState>("/chassis/state",10,std::bind(&Steer::getSteerAngleCallback,this,std::placeholders::_1));
            //不知道同步效果怎么样 todo 测一下
            //对于双输入的节点来看
            timer = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&Steer::solverCallback,this));

        }
        Steer::~Steer(){};
        void Steer::solverCallback()
        {
              
           
        }

        void Steer::getSetValueCallback(const rm_interfaces::msg::MasterChassisCmd::SharedPtr msg)
        {
            input.x_set = msg->x_set;
            input.y_set = msg->y_set;
            input.yaw_set = msg->yaw_set;
        }

        void Steer::getSteerAngleCallback(const rm_interfaces::msg::ChassisState::SharedPtr msg)
        {
            
        }



        
        void Steer::getParams()
        {
             
        }

        
    }
}

