#pragma once
#ifndef LOW_LEVEL_CTRL_H
#define LOW_LEVEL_CTRL_H
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "common/motor_crc.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "common/cxxopts.hpp"

using namespace std;

class LowLevelControl : public rclcpp::Node
{
public:
    LowLevelControl();
    // ~LowLevelControl();

private:
    void init_cmd();
    void state_callback(unitree_go::msg::LowState::SharedPtr msg);
    void joy_callback(unitree_go::msg::WirelessController::SharedPtr msg);
    void target_pos_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void state_machine();
    void state_transform(vector<double> &target_angels);
    void state_obs();
    void run_policy();
    static double jointLinearInterpolation(double initPos, double targetPos, double rate);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pos_puber_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_suber_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr joy_suber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_pos_suber_;

    unitree_go::msg::LowCmd cmd_msg_;
    std_msgs::msg::Float32MultiArray pos_data_;
    unitree_go::msg::MotorState motor[12];
    std_msgs::msg::Float32MultiArray torque_msg_;
    std_msgs::msg::Float32MultiArray rl_target_pos_;
    vector<int> sequence= {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};

    int rate_count_ = 0;
    int motion_time_ = 0;
    bool recieved_data_=false;
    bool is_uncontrolled_ = true;
    bool is_laydown_ = false;
    bool is_standing_ = false;
    bool should_laydown_ = false;
    bool should_stand_ = false;
    bool should_run_policy_=false;
    bool is_simulation;
    vector<double> q_init_=vector<double>(12,0);
    vector<double> kp=vector<double>(12,0);
    vector<double> kd=vector<double>(12,0);
    vector<double> standing_angels_ = {-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1, -1.5};
    vector<double> laydown_angels_ = {
        -0.03252546489238739,
        1.263495683670044,
        -2.8056771755218506,
        0.05084329843521118,
        1.248689889907837,
        -2.804839849472046,
        -0.3643723130226135,
        1.2936768531799316,
        -2.819483518600464,
        0.34015029668807983,
        1.2838366031646729,
        -2.8084890842437744};

    vector<double> q_des_=vector<double>(12,0);
};
#endif