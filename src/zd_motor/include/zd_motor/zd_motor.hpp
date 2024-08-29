#ifndef __ZD_MOTOR_HPP__
#define __ZD_MOTOR_HPP__

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "std_msgs/msg/int32.hpp"

#include "can_interfaces/msg/vci_can_obj.hpp"
using std::placeholders::_1;

using UInt8 = std_msgs::msg::UInt8;
using UInt16 = std_msgs::msg::UInt16;
using UInt32 = std_msgs::msg::UInt32;
using Int32 = std_msgs::msg::Int32;
using VciCanObjMsg = can_interfaces::msg::VciCanObj;
using ZdState = Zd_interfaces::msg::ZdState;

using namespace std::chrono_literals;

class ZdMotor : public rclcpp::Node
{
public:
    ZdMotor(rclcpp::NodeOptions options);

    bool send_can_req_msg(VciCanObjMsg msg);
    void can_msg_cb(const std::shared_ptr<VciCanObjMsg> msg);

    void rotate_dir_req_cb(const std::shared_ptr<UInt8> msg);
    void speed_req_cb(const std::shared_ptr<UInt16> msg);

    void pub_motor_state();

private:
    uint8_t node_id_;

    std::mutex mutex_;

    ZdState motor_state_;

    std::shared_ptr<rclcpp::Publisher<VciCanObjMsg>> pub_can_req_;
    std::shared_ptr<rclcpp::Subscription<VciCanObjMsg>> sub_can_msg_;
    
    std::shared_ptr<rclcpp::CallbackGroup> motor_state_cb_group_;
    std::shared_ptr<rclcpp::TimerBase> pub_motor_state_timer_;
    std::shared_ptr<rclcpp::Publisher<ZdState>> pub_motor_state_;

    std::shared_ptr<rclcpp::Subscription<UInt16>> sub_mode_req_;
    std::shared_ptr<rclcpp::Subscription<UInt16>> sub_controlword_req_;

    std::shared_ptr<rclcpp::Subscription<UInt8>> sub_rotate_dir_req_;
    std::shared_ptr<rclcpp::Subscription<UInt16>> sub_speed_req_;

    std::shared_ptr<rclcpp::Subscription<Int32>> sub_target_pos_req_;
    std::shared_ptr<rclcpp::Subscription<UInt32>> sub_profile_speed_req_;
};


#endif