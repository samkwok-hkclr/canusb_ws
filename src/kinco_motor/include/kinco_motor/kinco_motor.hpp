#ifndef __KINCO_MOTOR_HPP__
#define __KINCO_MOTOR_HPP__

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "std_msgs/msg/int32.hpp"

#include "can_interfaces/msg/vci_can_obj.hpp"
#include "kinco_interfaces/msg/kinco_state.hpp"

using std::placeholders::_1;

using UInt8 = std_msgs::msg::UInt8;
using UInt16 = std_msgs::msg::UInt16;
using UInt32 = std_msgs::msg::UInt32;
using Int32 = std_msgs::msg::Int32;
using VciCanObjMsg = can_interfaces::msg::VciCanObj;
using KincoState = kinco_interfaces::msg::KincoState;

using namespace std::chrono_literals;

class KincoMotor : public rclcpp::Node
{
public:
    KincoMotor(rclcpp::NodeOptions options);
    bool map_pdos();
    void map_tx1();
    void map_tx2();

    bool send_can_req_msg(VciCanObjMsg msg);
    void can_msg_cb(const std::shared_ptr<VciCanObjMsg> msg);

    void handle_tx1_pdo(const std::shared_ptr<VciCanObjMsg> msg);
    void handle_tx2_pdo(const std::shared_ptr<VciCanObjMsg> msg);
    void handle_nmt_control(const std::shared_ptr<VciCanObjMsg> msg);

    void set_heartbeat(uint16_t ms);

    void mode_req_cb(const std::shared_ptr<UInt16> msg);
    void controlword_req_cb(const std::shared_ptr<UInt16> msg); 

    void rotate_dir_req_cb(const std::shared_ptr<UInt8> msg);
    void speed_req_cb(const std::shared_ptr<UInt16> msg);

    void target_pos_req_cb(const std::shared_ptr<Int32> msg); 
    void profile_speed_req_cb(const std::shared_ptr<UInt32> msg); 

    void pub_motor_state();

private:
    uint8_t node_id_;

    const uint16_t tx1_cod_id = 0x180;
    const uint8_t tx1_inhibit_time = 0x0;
    const uint8_t tx1_event_time = 0xC8;
    const uint8_t group_tx1_pdo = 4;

    const uint16_t tx2_cod_id = 0x280;
    const uint8_t tx2_inhibit_time = 0x0;
    const uint8_t tx2_event_time = 0xC8;
    const uint8_t group_tx2_pdo = 2;

    // FIXME: setup config file for peak_current
    const int8_t peak_current = 8; // for FD114S Controller ONLY!!!!!!!!!!
    const float current_1A = 2048.0 / ((float) peak_current / 1.414214);

    std::mutex mutex_;

    KincoState motor_state_;

    std::shared_ptr<rclcpp::Publisher<VciCanObjMsg>> pub_can_req_;
    std::shared_ptr<rclcpp::Subscription<VciCanObjMsg>> sub_can_msg_;
    
    std::shared_ptr<rclcpp::CallbackGroup> motor_state_cb_group_;
    std::shared_ptr<rclcpp::TimerBase> pub_motor_state_timer_;
    std::shared_ptr<rclcpp::Publisher<KincoState>> pub_motor_state_;

    std::shared_ptr<rclcpp::Subscription<UInt16>> sub_mode_req_;
    std::shared_ptr<rclcpp::Subscription<UInt16>> sub_controlword_req_;

    std::shared_ptr<rclcpp::Subscription<UInt8>> sub_rotate_dir_req_;
    std::shared_ptr<rclcpp::Subscription<UInt16>> sub_speed_req_;

    std::shared_ptr<rclcpp::Subscription<Int32>> sub_target_pos_req_;
    std::shared_ptr<rclcpp::Subscription<UInt32>> sub_profile_speed_req_;
};

#endif
