#include "zyz_ik/arm.hpp"

#include <string>

namespace
{
    inline double angleDiff(double from, double to)
    {
        double d = to - from;
        while (d > M_PI)
            d -= 2 * M_PI;
        while (d < -M_PI)
            d += 2 * M_PI;
        return d;
    }

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
            target_R = q.toRotationMatrix();

            target_pose_.updated = true;
        }

        void ZYZIK::update()
        {

            double elapsed = (this->now() - transition_start_time_).seconds();
            double s = (transition_duration_ > 0 && elapsed < transition_duration_) ? elapsed / transition_duration_ : 1.0;
            double f = (3.0 - 2.0 * s) * s * s;

            double temp_joint_datla = 0.0;
            double temp_length = 0.0;
            double temp_j4 = 0;

            double total_j4 = 0;

            auto interp = [&](double from, double to)
            {
                return from + angleDiff(from, to) * f;
            };
            current_joint_angles_.joint_1 = target_joint_angles_.joint_1;
            current_joint_angles_.joint_2 =target_joint_angles_.joint_2;
            current_joint_angles_.joint_3 = target_joint_angles_.joint_3;
            current_joint_angles_.joint_4 = target_joint_angles_.joint_4;
            current_joint_angles_.joint_5 = target_joint_angles_.joint_5;
            current_joint_angles_.joint_6 = target_joint_angles_.joint_6;

            if (target_pose_.updated)
            {

                target_pose_.updated = false;

                j2LimitCalculate();

                double j[6] = {
                    target_joint_angles_.joint_1, target_joint_angles_.joint_2,
                    target_joint_angles_.joint_3, target_joint_angles_.joint_4,
                    target_joint_angles_.joint_5, target_joint_angles_.joint_6};

                RCLCPP_INFO(get_logger(), "target_joint_angles1: j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f j6=%.3f",
                            target_joint_angles_.joint_1, target_joint_angles_.joint_2,
                            target_joint_angles_.joint_3, target_joint_angles_.joint_4,
                            target_joint_angles_.joint_5, target_joint_angles_.joint_6);

                double x_act, y_act, z_act;
                computeFK(j, x_act, y_act, z_act);

                temp_joint_datla = M_PI / 2 - 0.21310470167 - current_joint_angles_.joint_3;
                max_angle = M_PI / 2 - 0.21310470167 - joint_limits_.joint_3.lower;
                max_length = sqrtf(powf(dh_params_.l2, 2) + powf(dh_params_.l3, 2) - 2 * dh_params_.l2 * dh_params_.l3 * cosf(max_angle));
                temp_length = sqrtf(powf(dh_params_.l2, 2) + powf(dh_params_.l3, 2) - 2 * dh_params_.l2 * dh_params_.l3 * cosf(temp_joint_datla));
                RCLCPP_INFO(get_logger(), "length: max_length=%.5f temp_length=%.5f joint_limits_.joint_3.lower =  %.5f ", max_length, temp_length, joint_limits_.joint_3.lower);

                RCLCPP_INFO(get_logger(), "target_pose1: x=%.5f y=%.5f z=%.5f", target_pose_.x, target_pose_.y, target_pose_.z);
                if (max_length > temp_length)
                {
                    target_pose_.x += x_act;
                    target_pose_.y += y_act;
                    target_pose_.z += z_act;
                }
                else
                {
                    target_pose_.x = x_act;
                    target_pose_.y = y_act;
                    target_pose_.z = z_act;
                }

                RCLCPP_INFO(get_logger(), "act: x=%.5f y=%.5f z=%.5f", x_act, y_act, z_act);
                RCLCPP_INFO(get_logger(), "target_pose2: x=%.5f y=%.5f z=%.5f", target_pose_.x, target_pose_.y, target_pose_.z);

                //     target_pose_.x = x_act;
                // target_pose_.y = y_act;
                // target_pose_.z = z_act;
                computeIK();
                start_joint_angles_ = current_joint_angles_;
                transition_start_time_ = this->now();
                
            }

            temp_j4 = current_joint_angles_.joint_4 - last_j4_angle;
            if (fabs(temp_j4) > M_PI)
            {
                if(temp_j4 > 0)
                {
                    temp_j4 =  2*M_PI - temp_j4; 
                }
                else
                {
                    temp_j4 = -2*M_PI - temp_j4;
                }

            }
            last_j4_angle = current_joint_angles_.joint_4;
            total_j4+=temp_j4;

            //     target_pose_.x = 0.012;
            //     target_pose_.y = 0;
            //     target_pose_.z = 0.074;
            //     target_pose_.roll = 0.0f;
            //     target_pose_.pitch = M_PI/4;
            //     target_pose_.yaw = 0;
            //     target_R = (Eigen::AngleAxisd(target_pose_.yaw, Eigen::Vector3d::UnitZ()) *
            //                 Eigen::AngleAxisd(target_pose_.pitch, Eigen::Vector3d::UnitY()) *
            //                 Eigen::AngleAxisd(target_pose_.roll, Eigen::Vector3d::UnitX())).matrix();
            //     computeIK();
            //     double j[6] = {
            //    target_joint_angles_.joint_1, target_joint_angles_.joint_2,
            //                 target_joint_angles_.joint_3, target_joint_angles_.joint_4,
            //                 target_joint_angles_.joint_5, target_joint_angles_.joint_6};

            //      RCLCPP_INFO(get_logger(), "current_joint_angles: j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f j6=%.3f",
            //                 current_joint_angles_.joint_1, current_joint_angles_.joint_2,
            //     current_joint_angles_.joint_3, current_joint_angles_.joint_4,
            //     current_joint_angles_.joint_5, current_joint_angles_.joint_6);

            //         double x_act, y_act, z_act;
            //     computeFK(j, x_act, y_act, z_act);

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

            double l2 = dh_params_.l2;

            // Eigen::Matrix3d R = (Eigen::AngleAxisd(w, Eigen::Vector3d::UnitZ()) *
            //                      Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
            //                      Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
            //                         .matrix();

            Eigen::Matrix4d Timu2base = Eigen::Matrix4d::Identity();
            Timu2base.block<3, 3>(0, 0) = target_R;
            Timu2base.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

            // RCLCPP_INFO_STREAM(this->get_logger(), "Timu2base:\n" << Timu2base);

            Eigen::Matrix4d T60 = Timu2base * Tend26_inv_;
            // RCLCPP_INFO_STREAM(this->get_logger(), "T60:\n" << T60);
            double j1 = 0;
            if (T60(0, 3) < 0)
                j1 = atan2f(-T60(1, 3), -T60(0, 3));
            else
                j1 = atan2f(T60(1, 3), T60(0, 3));

            double pho = hypotf(T60(0, 3), T60(1, 3));
            double c = hypotf(pho, T60(2, 3));
            double cj3 = (l2 * l2 + l4d4_across_ - c * c) / (2.0 * l2 * sqrtf(l4d4_across_));
            double j3 = -acosf(std::clamp(cj3, -1.0, 1.0)) + angle_l4d4_;

            double cj2 = (c * c + l2 * l2 - l4d4_across_) / (2.0 * c * l2);
            double j2_1 = acosf(std::clamp(cj2, -1.0, 1.0));
            double j2_2 = atan2f(T60(2, 3), pho);
            double j2 = 0;
            if (T60(0, 3) < 0)
            {
                // 腕部在基座 X 负半轴：angle(Rx(pi/2)*p40) = -pi + j2_2，因为atan2f得到角度为正半轴所以要加-pi取反，得到夹角
                j2 = -M_PI + j2_2 - j2_1;
            }
            else
            {
                // 腕部在基座 X 正半轴：angle(Rx(pi/2)*p40) = -j2_2
                j2 = -j2_2 - j2_1;
            }

            //@TODO: 这里的j2计算在某些位置可能会有问题，尤其是当腕部在基座X轴附近时，可能会出现数值不稳定或者多解的情况，需要进一步分析和测试。
            //就比如当x在负半轴时，肘部会有上折或者左折，下折情况，你该怎么选择呢
           


            Eigen::Matrix4d T10 = Eigen::Matrix4d::Identity();
            T10.block<3, 3>(0, 0) = Eigen::AngleAxisd(j1, Eigen::Vector3d::UnitZ()).matrix();

            Eigen::Matrix4d T21 = Eigen::Matrix4d::Identity();
            T21.block<3, 3>(0, 0) = (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
                                     Eigen::AngleAxisd(j2, Eigen::Vector3d::UnitZ()))
                                        .matrix();

            Eigen::Matrix4d T32 = Eigen::Matrix4d::Identity();
            T32.block<3, 3>(0, 0) = Eigen::AngleAxisd(j3, Eigen::Vector3d::UnitZ()).matrix();
            T32.block<3, 1>(0, 3) = Eigen::Vector3d(l2, 0, 0);

            Eigen::Matrix4d T43 = Eigen::Matrix4d::Identity();
            T43.block<3, 3>(0, 0) = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).matrix();
            T43.block<3, 1>(0, 3) = Eigen::Vector3d(dh_params_.deta_a, dh_params_.l3, 0);

            Eigen::Matrix3d R40 = (T10 * T21 * T32 * T43).block<3, 3>(0, 0);

            Eigen::Matrix3d R60 = T60.block<3, 3>(0, 0);
            Eigen::Matrix3d R64 = R40.transpose() * R60;

            Eigen::Matrix3d REND = R60 * T6t_.block<3, 3>(0, 0);

            RCLCPP_INFO_STREAM(this->get_logger(), "REND:\n"
                                                       << REND);

            Eigen::Vector3d temp_pos = R64  * Eigen::Vector3d(0, 0,1);
            

            //zyz解法
            // bool tool_up = temp_pos(0) <= 0;
    
            // RCLCPP_INFO_STREAM(this->get_logger(), "temp_pos:\n"
            //                                            << temp_pos);
            // double sin_beta_mag = hypotf(R64(2, 0), R64(2, 1));
            // double cos_beta = R64(2, 2);
            // double sin_beta = tool_up ? -sin_beta_mag : sin_beta_mag;

            // double alpha, gamma, beta;


            // beta = atan2f(sin_beta_mag, cos_beta);
            
            // if (fabs(beta) < EQS_VAL)
            // {
               
                
            //      gamma = atan2f(-R64(0, 1), R64(0, 0));
            // }
            // else if (fabs(fabs(beta) - M_PI) < EQS_VAL)
            // {
                
                
            //     gamma = atan2f(R64(0, 1), -R64(0, 0));
            // }
            // else
            // {
            //     alpha = atan2f(R64(1, 2), R64(0, 2));
            //     gamma = atan2f(R64(2, 1), -R64(2, 0));
            // }

            double pitch = 0, roll = 0, yaw = 0;
            pitch = atan2f(-R64(2, 0),hypotf(R64(0,0),R64(1,0)));

            if(fabs(fabs(pitch) - M_PI/2) < EQS_VAL)
            {
                yaw = 0;
                if(pitch > 0)
                {
                    roll = atan2f(R64(0, 1), R64(1, 1));
                }
                else
                {
                    roll = -atan2f(R64(0, 1), -R64(1, 1));
                }
            }
            else
            {
                roll = atan2f(R64(1, 0), R64(0, 0));
                yaw = atan2f(R64(2, 1), -R64(2, 2));
            }

            pitch = pitch + M_PI/2;

             
            // 先钳位再存储，确保 last_joint_angles_ 反映实际指令值
            target_joint_angles_.joint_1 = std::clamp(j1, joint_limits_.joint_1.lower, joint_limits_.joint_1.upper);
            target_joint_angles_.joint_2 = std::clamp(j2, joint_limits_.joint_2.lower, joint_limits_.joint_2.upper);
            target_joint_angles_.joint_3 = std::clamp(j3, joint_limits_.joint_3.lower, joint_limits_.joint_3.upper);
            target_joint_angles_.joint_4 = std::clamp(yaw, joint_limits_.joint_4.lower, joint_limits_.joint_4.upper);
            target_joint_angles_.joint_5 = std::clamp(pitch, joint_limits_.joint_5.lower, joint_limits_.joint_5.upper);
            target_joint_angles_.joint_6 = std::clamp(roll, joint_limits_.joint_6.lower, joint_limits_.joint_6.upper);

            RCLCPP_INFO(get_logger(), "target_joint_angles2: j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f j6=%.3f",
                        target_joint_angles_.joint_1, target_joint_angles_.joint_2,
                        target_joint_angles_.joint_3, target_joint_angles_.joint_4,
                        target_joint_angles_.joint_5, target_joint_angles_.joint_6);
        }

        void ZYZIK::computeFK(const double j[6], double &x, double &y, double &z)
        {
            double l2 = dh_params_.l2;

            // T10 = Rz(j1)
            Eigen::Matrix4d T10 = Eigen::Matrix4d::Identity();
            T10.block<3, 3>(0, 0) = Eigen::AngleAxisd(j[0], Eigen::Vector3d::UnitZ()).matrix();
            // RCLCPP_INFO_STREAM(this->get_logger(), "T10:\n" << T10);

            // T21 = Rz(j2) * Rx(-pi/2)
            Eigen::Matrix4d T21 = Eigen::Matrix4d::Identity();
            T21.block<3, 3>(0, 0) = (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
                                     Eigen::AngleAxisd(j[1], Eigen::Vector3d::UnitZ()))
                                        .matrix();

            // T32 = Rz(j3) * Tx(l2, 0, 0)
            Eigen::Matrix4d T32 = Eigen::Matrix4d::Identity();
            T32.block<3, 3>(0, 0) = Eigen::AngleAxisd(j[2], Eigen::Vector3d::UnitZ()).matrix();
            T32.block<3, 1>(0, 3) = Eigen::Vector3d(l2, 0, 0);

            // T43 = Rx(-pi/2) with translation (deta_a, l3, 0)
            Eigen::Matrix4d T43 = Eigen::Matrix4d::Identity();
            T43.block<3, 3>(0, 0) = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).matrix();
            T43.block<3, 1>(0, 3) = Eigen::Vector3d(dh_params_.deta_a, dh_params_.l3, 0);

            Eigen::Matrix4d T40 = T10 * T21 * T32 * T43;
            Eigen::Matrix3d R40 = T40.block<3, 3>(0, 0);
            Eigen::Vector3d p40 = T40.block<3, 1>(0, 3);

            // wrist rotation = Rz(j4) * Ry(-j5) * Rz(j6)
            Eigen::Matrix3d R64 = (Eigen::AngleAxisd(j[3], Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(-j[4], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(j[5], Eigen::Vector3d::UnitZ()))
                                      .matrix();

            // T60 = T40 * T_wrist (wrist joints co-located, so p60 = p40)
            Eigen::Matrix3d R60 = R40 * R64;
            // T_tool = T60 * T6t
            Eigen::Vector3d pt = p40 + R60 * T6t_.block<3, 1>(0, 3);

            Eigen::Matrix3d Re0 = R40 * R64 * T6t_.block<3, 3>(0, 0);

            // Manual RPY extraction (consistent with computeIK)
            double rpy_p = asin(-Re0(2, 0));
            double rpy_y, rpy_r;
            if (cos(rpy_p) > EQS_VAL) {
                rpy_y = atan2(Re0(1, 0), Re0(0, 0));
                rpy_r = atan2(Re0(2, 1), Re0(2, 2));
            } else {
                rpy_y = atan2(-Re0(0, 1), Re0(1, 1));
                rpy_r = 0.0;
            }
            Eigen::Vector3d rpy(rpy_y, rpy_p, rpy_r);

            RCLCPP_INFO_STREAM(this->get_logger(), "Re0:\n"
                                                       << Re0);
            RCLCPP_INFO_STREAM(this->get_logger(), "FK RPY:\n" << rpy);
                                                       
            x = pt(0);
            y = pt(1);
            z = pt(2);
            RCLCPP_INFO_STREAM(this->get_logger(), "p:\n"
                                                       << pt);
        }

        void ZYZIK::j2LimitCalculate()
        {
            joint_limits_.joint_3.lower = M_PI / 2 - (0.21310470167 + (0.75049157836 + (current_joint_angles_.joint_2 + M_PI - 0.24434609528)));
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

            double l2 = dh_params_.l2, l3 = dh_params_.l3, da = dh_params_.deta_a;
            l4d4_across_ = da * da + l3 * l3;
            if (fabs(da) > 1e-6)
            {
                angle_l4d4_ = atan2f(l3, -da);
            }
            else
            {
                angle_l4d4_ = M_PI_2;
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
            current_joint_angles_.joint_1 = std::clamp(
                this->get_parameter("initial_angles.joint_1").as_double(),
                joint_limits_.joint_1.lower, joint_limits_.joint_1.upper);
            current_joint_angles_.joint_2 = std::clamp(
                this->get_parameter("initial_angles.joint_2").as_double(),
                joint_limits_.joint_2.lower, joint_limits_.joint_2.upper);
            current_joint_angles_.joint_3 = std::clamp(
                this->get_parameter("initial_angles.joint_3").as_double(),
                joint_limits_.joint_3.lower, joint_limits_.joint_3.upper);
            current_joint_angles_.joint_4 = std::clamp(
                this->get_parameter("initial_angles.joint_4").as_double(),
                joint_limits_.joint_4.lower, joint_limits_.joint_4.upper);
            current_joint_angles_.joint_5 = std::clamp(
                this->get_parameter("initial_angles.joint_5").as_double(),
                joint_limits_.joint_5.lower, joint_limits_.joint_5.upper);
            current_joint_angles_.joint_6 = std::clamp(
                this->get_parameter("initial_angles.joint_6").as_double(),
                joint_limits_.joint_6.lower, joint_limits_.joint_6.upper);
            RCLCPP_INFO(this->get_logger(), "Initial angles: %.2f %.2f %.2f %.2f %.2f %.2f",
                        current_joint_angles_.joint_1, current_joint_angles_.joint_2,
                        current_joint_angles_.joint_3, current_joint_angles_.joint_4,
                        current_joint_angles_.joint_5, current_joint_angles_.joint_6);

            // initialize targets to current angles (no transition on startup)
            target_joint_angles_ = current_joint_angles_;
            start_joint_angles_ = current_joint_angles_;

            this->declare_parameter("transition_duration", 0.08);
            transition_duration_ = this->get_parameter("transition_duration").as_double();
            RCLCPP_INFO(this->get_logger(), "Transition duration: %.3f s", transition_duration_);

            this->declare_parameter("eqs_val", 1e-6);
            EQS_VAL = get_parameter("eqs_val").as_double();

            this->declare_parameter("dh_params.tool_link", 0.0f);
            tool_link = get_parameter("dh_params.tool_link").as_double();

            this->declare_parameter("joint3_run_time_zero", -0.6981317008f);
            joint3_zero = get_parameter("joint3_run_time_zero").as_double();

            // pre-compute Tend26_inv (tool->joint6 inverse)
            //绕x轴旋转180度，工具坐标系z轴朝下，x轴朝前，y轴朝右
            Eigen::Matrix4d T6t = Eigen::Matrix4d::Identity();
            T6t.block<3, 3>(0, 0) << 1, 0, 0,
                                     0, -1, 0,
                                     0, 0, -1;

                //这里要加上
            T6t.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, 0);
            T6t_ = T6t;

            Tend26_inv_ = T6t.inverse();
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(solver::arm::ZYZIK)
