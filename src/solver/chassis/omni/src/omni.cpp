#include "omni/omni.hpp"

namespace solver
{
    namespace chassis
    {
      Omni::Omni(const rclcpp::NodeOptions &options):Node("omni_solver",options)
      {
          RCLCPP_INFO(this->get_logger(),"this is Omni");
            getParams();
            solve_value_pub = this->create_publisher<rm_interfaces::msg::ChassisSolve>("/solver/omni",10);
            cmd_sub = this->create_subscription<rm_interfaces::msg::MasterChassisCmd>("/master/chassis/set",10,std::bind(&Omni::solverCallback,this,std::placeholders::_1));
      }
      Omni::~Omni(){};
      
      void Omni::solverCallback(const rm_interfaces::msg::MasterChassisCmd::SharedPtr msg)
      {
        auto solve_value = rm_interfaces::msg::ChassisSolve();
        input.x_set = msg->x_set;
        input.y_set = msg->y_set;
        input.yaw_set = msg->yaw_set;

        //四轮长宽都是相等的,线速度
        solve_value.lf = output.lf = (-input.x_set - input.y_set)/sqrt(2) + input.yaw_set;
        solve_value.ld = output.rf = (input.x_set - input.y_set)/sqrt(2)  + input.yaw_set;
        solve_value.rf = output.ld = (input.x_set - input.y_set)/sqrt(2) + input.yaw_set;
        solve_value.rd = output.rd = (-input.x_set - input.y_set)/sqrt(2) + input.yaw_set;

        solve_value_pub->publish(solve_value);
      }

      void Omni::getParams()
      {

      }
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(solver::chassis::Omni)