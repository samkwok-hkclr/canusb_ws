#ifndef __CAN_TRANSCEIVER_HPP__
#define __CAN_TRANSCEIVER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include <algorithm>
#include <iterator>
#include <chrono>

#include "controlcan.h"

#include "can_interfaces/msg/vci_can_obj.hpp"

using std::placeholders::_1;

using VciCanObjMsg = can_interfaces::msg::VciCanObj;

using namespace std::chrono_literals;

class CanTransceiver : public rclcpp::Node
{
public:
    CanTransceiver(rclcpp::NodeOptions options);

    bool find_canusb();
    bool open_canusb();
    bool read_canusb();
    bool init_canusb();
    bool start_canusb();

    void can_msg_cb();
    void can_req_cb(const std::shared_ptr<VciCanObjMsg> msg);

private:
    const int rec_buf_len = 3000;
    const int can_index = 0;
    uint16_t can_count = 0;

    VCI_BOARD_INFO pInfo1[50];
    VCI_BOARD_INFO pInfo;
    VCI_CAN_OBJ rec[3000];

    std::mutex mutex_;

    std::shared_ptr<rclcpp::CallbackGroup> pub_cb_group_;

    std::shared_ptr<rclcpp::TimerBase> pub_can_msg_timer_;

    std::shared_ptr<rclcpp::Publisher<VciCanObjMsg>> pub_can_msg_;

    std::shared_ptr<rclcpp::Subscription<VciCanObjMsg>> sub_can_req_;
};

#endif