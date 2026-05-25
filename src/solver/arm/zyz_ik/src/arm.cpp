#include "zyz_ik/arm.hpp"

#include <string>

namespace
{
    constexpr double PI = M_PI;
    constexpr double EQS_VAL = 1e-6;
    inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

namespace solver
{
    namespace arm
    {
        ZYZIK::ZYZIK(const rclcpp::NodeOptions &options) : Node("zyz_ik_solver", options)
        {
            RCLCPP_INFO(this->get_logger(), "ZYZ IK Solver started");
            getParams();

            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
            target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/target/pose", 10, std::bind(&ZYZIK::targetPoseCallback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ZYZIK::update, this));
        }

        ZYZIK::~ZYZIK() {}

        void ZYZIK::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            target_pose_.x = msg->pose.position.x;
            target_pose_.y = msg->pose.position.y;
            target_pose_.z = msg->pose.position.z;

            Eigen::Quaterniond q(msg->pose.orientation.w,
                                 msg->pose.orientation.x,
                                 msg->pose.orientation.y,
                                 msg->pose.orientation.z);
            Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(2, 1, 0);
            target_pose_.yaw   = rpy(0);
            target_pose_.pitch = rpy(1);
            target_pose_.roll  = rpy(2);

            target_pose_.updated = true;
        }

        void ZYZIK::update()
        {
            if (target_pose_.updated)
            {
                target_pose_.updated = false;

               
                
               

                // RCLCPP_INFO(get_logger(), "target_pose1: x=%.5f y=%.5f z=%.5f", x_act, y_act, z_act);
                // RCLCPP_INFO(get_logger(), "target_pose2: x=%.5f y=%.5f z=%.5f", target_pose_.x, target_pose_.y, target_pose_.z);
               
                // RCLCPP_INFO(get_logger(), "target_pose3: x=%.5f y=%.5f z=%.5f", target_pose_.x, target_pose_.y, target_pose_.z);

            }  double j[6] = {
                    current_joint_angles_.joint_1, current_joint_angles_.joint_2,
                    current_joint_angles_.joint_3, current_joint_angles_.joint_4,
                    current_joint_angles_.joint_5, current_joint_angles_.joint_6
                };
            double x_act, y_act, z_act;
            
            target_pose_.x = 0.200f;
            target_pose_.y = 0.000f;
            target_pose_.z = 0.300f;
            target_pose_.roll = 0;
            target_pose_.pitch = PI/3;
            target_pose_.yaw = 0;
            computeIK();
            // RCLCPP_INFO(get_logger(), "current_joint_angles: j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f j6=%.3f",
            //             current_joint_angles_.joint_1, current_joint_angles_.joint_2,
            //             current_joint_angles_.joint_3, current_joint_angles_.joint_4,
            //             current_joint_angles_.joint_5, current_joint_angles_.joint_6);

            computeFK(j, x_act, y_act, z_act);
            // RCLCPP_INFO(get_logger(), "target_pose1: x=%.5f y=%.5f z=%.5f", x_act, y_act, z_act);

            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = this->now();
            msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
            msg.position = {current_joint_angles_.joint_1, current_joint_angles_.joint_2,
                            current_joint_angles_.joint_3, current_joint_angles_.joint_4,
                            current_joint_angles_.joint_5, current_joint_angles_.joint_6};
            
            joint_state_pub_->publish(msg);
        }

        void ZYZIK::computeIK()
        {
            double x = target_pose_.x, y = target_pose_.y, z = target_pose_.z;
            double r = target_pose_.roll, p = target_pose_.pitch, w = target_pose_.yaw;
            double l2 = dh_params_.l2;

            // ---- 1. RPY -> rotation matrix (Rz*yaw * Ry*pitch * Rx*roll) ----
            Eigen::Matrix3d R = (Eigen::AngleAxisd(w, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).matrix();

            // ---- 2. homogeneous transform Timu2base = [R | t] ----
            Eigen::Matrix4d Timu2base = Eigen::Matrix4d::Identity();
            Timu2base.block<3,3>(0,0) = R;
            Timu2base.block<3,1>(0,3) = Eigen::Vector3d(x, y, z);

            //RCLCPP_INFO_STREAM(this->get_logger(), "Timu2base:\n" << Timu2base);

            // ---- 3. T60 = Timu2base * Tend26_inv ----
            Eigen::Matrix4d T60 = Timu2base * Tend26_inv_;
            //RCLCPP_INFO_STREAM(this->get_logger(), "T60:\n" << T60);

            // ---- 4. solve j1 ----
            double j1 = atan2f(T60(1,3), T60(0,3));

            // ---- 5. solve j3 (geometric) ----
            double pho = hypotf(T60(0,3), T60(1,3));
            double c = hypotf(pho, T60(2,3));
            double cj3 = (l2 * l2 + l4d4_across_ - c * c) / (2.0 * l2 * sqrtf(l4d4_across_));
            double j3 = -acosf(clamp(cj3, -1.0, 1.0)) + angle_l4d4_;

            // ---- 6. solve j2 (geometric) ----
            double cj2 = (c * c + l2 * l2 - l4d4_across_) / (2.0 * c * l2);
            double j2_1 = acosf(clamp(cj2, -1.0, 1.0));
            double j2_2 = atan2f(T60(2,3), pho);
            double j2 = -j2_2 - j2_1;

            // ---- 7. compute T40 through DH chain ----
            Eigen::Matrix4d T10 = Eigen::Matrix4d::Identity();
            T10.block<3,3>(0,0) = Eigen::AngleAxisd(j1, Eigen::Vector3d::UnitZ()).matrix();

            Eigen::Matrix4d T21 = Eigen::Matrix4d::Identity();
            T21.block<3,3>(0,0) = (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(j2, Eigen::Vector3d::UnitZ())).matrix();

            Eigen::Matrix4d T32 = Eigen::Matrix4d::Identity();
            T32.block<3,3>(0,0) = Eigen::AngleAxisd(j3, Eigen::Vector3d::UnitZ()).matrix();
            T32.block<3,1>(0,3) = Eigen::Vector3d(l2, 0, 0);

            Eigen::Matrix4d T43 = Eigen::Matrix4d::Identity();
            T43.block<3,3>(0,0) = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).matrix();
            T43.block<3,1>(0,3) = Eigen::Vector3d(dh_params_.deta_a, dh_params_.l3, 0);

            Eigen::Matrix3d R40 = (T10 * T21 * T32 * T43).block<3,3>(0,0);

            // ---- 8. R64 = R40^T * R60 (ZYZ decomposition) ----
            Eigen::Matrix3d R60 = T60.block<3,3>(0,0);
            Eigen::Matrix3d R64 = R40.transpose() * R60;
            

            RCLCPP_INFO_STREAM(this->get_logger(), "R64:\n" << R64);
           
            // ---- 9. solve j4, j5, j6 (ZYZ Euler angles from R64) ----
            double alpha, gamma;
            double beta = atan2f(hypotf(R64(2,0), R64(2,1)), R64(2,2));
            

            if (fabs(beta) < EQS_VAL)
            {
                alpha = 0;
                gamma = atan2f(-R64(0,1), R64(0,0));
            }
            else if (fabs(beta - PI) < EQS_VAL)
            {
                alpha = 0;
                gamma = atan2f(R64(0,1), -R64(0,0));
            }
            else
            {
                alpha = atan2f(R64(1,2), R64(0,2));
                gamma = atan2f(R64(2,1), -R64(2,0));
            }

            double j4 = alpha;
            double j5 = -beta;
            double j6 = gamma;

            RCLCPP_INFO_STREAM(this->get_logger(), "R64A:\n" << j4<<j5<<j6);

          
             
            // RCLCPP_INFO(get_logger(), "current_joint_angles: j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f j6=%.3f",
            //              current_joint_angles_.joint_1, current_joint_angles_.joint_2,
            //              current_joint_angles_.joint_3, current_joint_angles_.joint_4,
            //             current_joint_angles_.joint_5, current_joint_angles_.joint_6);

            // ---- 10. clamp to joint limits ----
            current_joint_angles_.joint_1 = clamp(j1, joint_limits_.joint_1.lower, joint_limits_.joint_1.upper);
            current_joint_angles_.joint_2 = clamp(j2, joint_limits_.joint_2.lower, joint_limits_.joint_2.upper);
            current_joint_angles_.joint_3 = clamp(j3, joint_limits_.joint_3.lower, joint_limits_.joint_3.upper);
            current_joint_angles_.joint_4 = clamp(j4, joint_limits_.joint_4.lower, joint_limits_.joint_4.upper);
            current_joint_angles_.joint_5 = clamp(j5, joint_limits_.joint_5.lower, joint_limits_.joint_5.upper);
            current_joint_angles_.joint_6 = clamp(j6, joint_limits_.joint_6.lower, joint_limits_.joint_6.upper);
         
        }

        void ZYZIK::computeFK(const double j[6], double &x, double &y, double &z)
        {
            double l2 = dh_params_.l2;

            // T10 = Rz(j1)
            Eigen::Matrix4d T10 = Eigen::Matrix4d::Identity();
            T10.block<3,3>(0,0) = Eigen::AngleAxisd(j[0], Eigen::Vector3d::UnitZ()).matrix();
           //RCLCPP_INFO_STREAM(this->get_logger(), "T10:\n" << T10);

            // T21 = Rz(j2) * Rx(-pi/2)
            Eigen::Matrix4d T21 = Eigen::Matrix4d::Identity();
            T21.block<3,3>(0,0) = (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(j[1], Eigen::Vector3d::UnitZ())).matrix();

            // T32 = Rz(j3) * Tx(l2, 0, 0)
            Eigen::Matrix4d T32 = Eigen::Matrix4d::Identity();
            T32.block<3,3>(0,0) = Eigen::AngleAxisd(j[2], Eigen::Vector3d::UnitZ()).matrix();
            T32.block<3,1>(0,3) = Eigen::Vector3d(l2, 0, 0);
            
            // T43 = Rx(-pi/2) with translation (deta_a, l3, 0)
            Eigen::Matrix4d T43 = Eigen::Matrix4d::Identity();
            T43.block<3,3>(0,0) = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).matrix();
            T43.block<3,1>(0,3) = Eigen::Vector3d(dh_params_.deta_a, dh_params_.l3, 0);
           
            Eigen::Matrix4d T40 = T10 * T21 * T32 * T43;
            Eigen::Matrix3d R40 = T40.block<3,3>(0,0);
            Eigen::Vector3d p40 = T40.block<3,1>(0,3);
           
        
            // wrist rotation = Rz(j4) * Ry(-j5) * Rz(j6)
            Eigen::Matrix3d R64 = (Eigen::AngleAxisd(j[3], Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(-j[4], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(j[5], Eigen::Vector3d::UnitZ())).matrix();
            RCLCPP_INFO_STREAM(this->get_logger(), "R64A:\n" << R64);

            // T60 = T40 * T_wrist (wrist joints co-located, so p60 = p40)
            Eigen::Matrix3d R60 = R40 * R64;
            // T_tool = T60 * T6t
            Eigen::Vector3d pt = p40 + R60 * T6t_.block<3,1>(0,3);


            Eigen::Matrix3d Re0 = R40 * R64 * T6t_.block<3,3>(0,0);
            Eigen::Vector3d rpy = Re0.eulerAngles(2,1,0);

            //  RCLCPP_INFO_STREAM(this->get_logger(), "Re0:\n" << Re0);
            //  RCLCPP_INFO_STREAM(this->get_logger(), "RPY:\n" << rpy);
            x = pt(0);
            y = pt(1);
            z = pt(2);
            //RCLCPP_INFO_STREAM(this->get_logger(), "p:\n" << pt);
        }

        void ZYZIK::getParams()
        {
            this->declare_parameter("dh_params.l2", 0.380);
            this->declare_parameter("dh_params.l3", 0.3783);
            this->declare_parameter("dh_params.deta_a", -0.055);
            this->declare_parameter("dh_params.deta_d", 0.0);

            dh_params_.l2 = this->get_parameter("dh_params.l2").as_double();
            dh_params_.l3 = this->get_parameter("dh_params.l3").as_double();
            dh_params_.deta_a = this->get_parameter("dh_params.deta_a").as_double();
            dh_params_.deta_d = this->get_parameter("dh_params.deta_d").as_double();

            RCLCPP_INFO(this->get_logger(), "DH: l2=%.3f l3=%.3f deta_a=%.3f deta_d=%.3f",
                        dh_params_.l2, dh_params_.l3, dh_params_.deta_a, dh_params_.deta_d);

            double l2 = dh_params_.l2, da = dh_params_.deta_a;
            if (fabs(da) > 1e-6)
            {
                angle_l4d4_ = atan2(l2, -da);
                l4d4_across_ = l2 * l2 + da * da;
            }
            else
            {
                angle_l4d4_ = 0;
                l4d4_across_ = l2 * l2;
            }

            // load joint limits
            for (int i = 1; i <= 6; ++i)
            {
                std::string key = "joint_limits.joint_" + std::to_string(i);
                this->declare_parameter(key + ".lower", 0.0);
                this->declare_parameter(key + ".upper", 0.0);
            }
            joint_limits_.joint_1.lower = this->get_parameter("joint_limits.joint_1.lower").as_double();
            joint_limits_.joint_1.upper = this->get_parameter("joint_limits.joint_1.upper").as_double();
            joint_limits_.joint_2.lower = this->get_parameter("joint_limits.joint_2.lower").as_double();
            joint_limits_.joint_2.upper = this->get_parameter("joint_limits.joint_2.upper").as_double();
            joint_limits_.joint_3.lower = this->get_parameter("joint_limits.joint_3.lower").as_double();
            joint_limits_.joint_3.upper = this->get_parameter("joint_limits.joint_3.upper").as_double();
            joint_limits_.joint_4.lower = this->get_parameter("joint_limits.joint_4.lower").as_double();
            joint_limits_.joint_4.upper = this->get_parameter("joint_limits.joint_4.upper").as_double();
            joint_limits_.joint_5.lower = this->get_parameter("joint_limits.joint_5.lower").as_double();
            joint_limits_.joint_5.upper = this->get_parameter("joint_limits.joint_5.upper").as_double();
            joint_limits_.joint_6.lower = this->get_parameter("joint_limits.joint_6.lower").as_double();
            joint_limits_.joint_6.upper = this->get_parameter("joint_limits.joint_6.upper").as_double();

            RCLCPP_INFO(this->get_logger(), "Joint limits loaded");

            // load initial angles
            for (int i = 1; i <= 6; ++i)
            {
                std::string key = "initial_angles.joint_" + std::to_string(i);
                this->declare_parameter(key, 0.0);
            }
            current_joint_angles_.joint_1 = clamp(
                this->get_parameter("initial_angles.joint_1").as_double(),
                joint_limits_.joint_1.lower, joint_limits_.joint_1.upper);
            current_joint_angles_.joint_2 = clamp(
                this->get_parameter("initial_angles.joint_2").as_double(),
                joint_limits_.joint_2.lower, joint_limits_.joint_2.upper);
            current_joint_angles_.joint_3 = clamp(
                this->get_parameter("initial_angles.joint_3").as_double(),
                joint_limits_.joint_3.lower, joint_limits_.joint_3.upper);
            current_joint_angles_.joint_4 = clamp(
                this->get_parameter("initial_angles.joint_4").as_double(),
                joint_limits_.joint_4.lower, joint_limits_.joint_4.upper);
            current_joint_angles_.joint_5 = clamp(
                this->get_parameter("initial_angles.joint_5").as_double(),
                joint_limits_.joint_5.lower, joint_limits_.joint_5.upper);
            current_joint_angles_.joint_6 = clamp(
                this->get_parameter("initial_angles.joint_6").as_double(),
                joint_limits_.joint_6.lower, joint_limits_.joint_6.upper);
            RCLCPP_INFO(this->get_logger(), "Initial angles: %.2f %.2f %.2f %.2f %.2f %.2f",
                current_joint_angles_.joint_1, current_joint_angles_.joint_2,
                current_joint_angles_.joint_3, current_joint_angles_.joint_4,
                current_joint_angles_.joint_5, current_joint_angles_.joint_6);

            // pre-compute Tend26_inv (tool->joint6 inverse)
            Eigen::Matrix4d T6t = Eigen::Matrix4d::Identity();
            T6t.block<3,3>(0,0) << 0, 0, 1,
                                    0, -1, 0,
                                   1, 0, 0;
            T6t.block<3,1>(0,3) = Eigen::Vector3d(0, 0, 0);
           
            Tend26_inv_ = T6t.inverse();
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(solver::arm::ZYZIK)
