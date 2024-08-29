
#include "kinco_motor/kinco_motor.hpp"

KincoMotor::KincoMotor(rclcpp::NodeOptions options)
    : Node("kinco_motor_node", options)
{
    this->declare_parameter("node_id", 0);

    node_id_ = this->get_parameter("node_id").as_int();
    while (node_id_ <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Node ID Error! Node ID: %d", node_id_);
        rclcpp::sleep_for(5s);
    }

    rclcpp::QoSInitialization can_qos_init = rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 128);
    rclcpp::QoS can_qos(can_qos_init);
    can_qos.reliable();
    can_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Do not add namespace to the topics can_req and can_msg
    pub_can_req_ = this->create_publisher<VciCanObjMsg>("/can_req", can_qos);
    sub_can_msg_ = this->create_subscription<VciCanObjMsg>("/can_msg", can_qos, std::bind(&KincoMotor::can_msg_cb, this, _1));

    motor_state_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_motor_state_timer_ = this->create_wall_timer(500ms, std::bind(&KincoMotor::pub_motor_state, this), motor_state_cb_group_);
    pub_motor_state_ = this->create_publisher<KincoState>("motor_state", 1);
    
    sub_mode_req_ = this->create_subscription<Int8>("mode_req", 1, std::bind(&KincoMotor::mode_req_cb, this, _1));
    sub_controlword_req_ = this->create_subscription<UInt16>("controlword_req", 1, std::bind(&KincoMotor::controlword_req_cb, this, _1));

    sub_rotate_dir_req_ = this->create_subscription<UInt8>("rotate_dir_req", 1, std::bind(&KincoMotor::rotate_dir_req_cb, this, _1));
    sub_speed_req_ = this->create_subscription<UInt16>("speed_req", 1, std::bind(&KincoMotor::speed_req_cb, this, _1));

    sub_target_pos_req_ = this->create_subscription<Int32>("target_pos_req", 1, std::bind(&KincoMotor::target_pos_req_cb, this, _1));
    sub_profile_speed_req_ = this->create_subscription<UInt32>("profile_speed_req", 1, std::bind(&KincoMotor::profile_speed_req_cb, this, _1));

    if (!map_pdos())
        RCLCPP_ERROR(this->get_logger(), "Map PDOs Error!");

    set_heartbeat(1000);
}

bool KincoMotor::map_pdos()
{
    RCLCPP_INFO(this->get_logger(), "Started to map the PDOs");

    map_tx1();
    map_tx2();

    return true;
}

void KincoMotor::map_tx1()
{   
    VciCanObjMsg msg;

    msg.id = 0x600 + node_id_;
    msg.data_len = 8;
    msg.data = {0x23, 0x00, 0x1A, 0x01, 0x10, 0x0, 0x40, 0x60};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Controlword mapped TX1_PDO1");
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data = {0x23, 0x00, 0x1A, 0x02, 0x08, 0x0, 0x60, 0x60};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Operation Mode mapped TX1_PDO2");
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data = {0x23, 0x00, 0x1A, 0x03, 0x10, 0x0, 0x41, 0x60};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Statusword mapped TX1_PDO3");
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data = {0x23, 0x00, 0x1A, 0x04, 0x10, 0x0, 0x78, 0x60};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Actual Current mapped TX1_PDO4");
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data = {0x23, 0x00, 0x18, 0x01, (uint8_t) ((tx1_cod_id & 0xFF) + node_id_), 0x01, 0x00, 0x00};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Set TX1 to COD-ID: 0x%03X", tx1_cod_id);
    rclcpp::sleep_for(10ms);
    
    msg.data.clear();
    msg.data_len = 5;
    msg.data = {0x2F, 0x00, 0x18, 0x02, 0xFE, 0x00, 0x00, 0x00};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Set TX1 to Async");
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data_len = 8;
    msg.data = {0x2B, 0x00, 0x18, 0x03, tx1_inhibit_time, 0x00, 0x00, 0x00};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Set TX1 inhibit time to %d ms", tx1_inhibit_time);
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data = {0x2B, 0x00, 0x18, 0x05, tx1_event_time, 0x00, 0x00, 0x00};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Set TX1 event time to %d ms", tx1_event_time);
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data = {0x2F, 0x00, 0x1A, 0x0, group_tx1_pdo, 0x00, 0x00, 0x00};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Set Group TX1 PDO: %d", group_tx1_pdo);
    rclcpp::sleep_for(10ms);
}

void KincoMotor::map_tx2()
{
    VciCanObjMsg msg;

    msg.id = 0x600 + node_id_;
    msg.data_len = 8;
    msg.data = {0x23, 0x01, 0x1A, 0x01, 0x20, 0x0, 0x63, 0x60};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Actual Position mapped TX2_PDO1");
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data = {0x23, 0x01, 0x1A, 0x02, 0x20, 0x0, 0x6C, 0x60};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Actual Rotate Speed mapped TX2_PDO1");
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data = {0x23, 0x01, 0x18, 0x01, (uint8_t) ((tx2_cod_id & 0xFF) + node_id_), 0x02, 0x00, 0x00};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Set TX2 to COD-ID: 0x%03X", tx2_cod_id);
    rclcpp::sleep_for(10ms);
    
    msg.data.clear();
    msg.data_len = 5;
    msg.data = {0x2F, 0x01, 0x18, 0x02, 0xFE, 0x00, 0x00, 0x00};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Set TX2 to Async");
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data_len = 8;
    msg.data = {0x2B, 0x01, 0x18, 0x03, tx2_inhibit_time, 0x00, 0x00, 0x00};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Set TX2 inhibit time to %d ms", tx2_inhibit_time);
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data = {0x2B, 0x01, 0x18, 0x05, tx2_event_time, 0x00, 0x00, 0x00};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Set TX2 event time to %d ms", tx2_event_time);
    rclcpp::sleep_for(10ms);

    msg.data.clear();
    msg.data = {0x2F, 0x01, 0x1A, 0x0, group_tx2_pdo, 0x00, 0x00, 0x00};
    pub_can_req_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Set Group TX2 PDO: %d", group_tx2_pdo);
    rclcpp::sleep_for(10ms);
}

void KincoMotor::can_msg_cb(const std::shared_ptr<VciCanObjMsg> msg)
{
    if ((msg->id & 0x7F) != node_id_)
        return;
     
    switch (msg->id & 0x780)
    {
        case 0x180:
            handle_tx1_pdo(msg);
            break;
        case 0x280:
            handle_tx2_pdo(msg);
            break;
        case 0x700:
            handle_nmt_control(msg);
            break;
    }
}

void KincoMotor::handle_tx1_pdo(const std::shared_ptr<VciCanObjMsg> msg)
{
    const std::lock_guard<std::mutex> lock(this->mutex_);

    motor_state_.controlword = (static_cast<uint16_t>(msg->data[1]) << 8) |
                               (static_cast<uint16_t>(msg->data[0]));

    motor_state_.operation_mode = (static_cast<uint8_t>(msg->data[2]));

    motor_state_.statusword = (static_cast<uint16_t>(msg->data[4]) << 8) |
                              (static_cast<uint16_t>(msg->data[3]));

    uint16_t i_DEC = (static_cast<uint16_t>(msg->data[6]) << 8) |
                     (static_cast<uint16_t>(msg->data[5]));
    
    if (i_DEC > 0xFF00)
        motor_state_.actual_current = 0.0;
    else
    {
        motor_state_.actual_current = (float) ((float) i_DEC / current_1A);
    }
}

void KincoMotor::handle_tx2_pdo(const std::shared_ptr<VciCanObjMsg> msg)
{
    int32_t speed_DEC = (static_cast<uint32_t>(msg->data[7]) << 24) |
                        (static_cast<uint32_t>(msg->data[6]) << 16) |
                        (static_cast<uint32_t>(msg->data[5]) << 8) |
                        (static_cast<uint32_t>(msg->data[4]));

    const std::lock_guard<std::mutex> lock(this->mutex_);
    motor_state_.actual_speed = (int32_t) ((float) speed_DEC * 1875.0f / 512.0f / 10000.0f);

    motor_state_.actual_position = (static_cast<uint32_t>(msg->data[3]) << 24) |
                                   (static_cast<uint32_t>(msg->data[2]) << 16) |
                                   (static_cast<uint32_t>(msg->data[1]) << 8) |
                                   (static_cast<uint32_t>(msg->data[0]));
}

void KincoMotor::handle_nmt_control(const std::shared_ptr<VciCanObjMsg> msg)
{
    (void) msg;
    const std::lock_guard<std::mutex> lock(this->mutex_);
    motor_state_.heartbeat_timestamp = rclcpp::Clock().now().seconds();
}

void KincoMotor::set_heartbeat(uint16_t ms)
{
    VciCanObjMsg can_msg;

    can_msg.id = 0x600 + node_id_;
    can_msg.data_len = 6;
    can_msg.data = {0x2B, 0x17, 0x10, 0x00, static_cast<uint8_t>(ms & 0xFF), 
                                            static_cast<uint8_t>((ms >> 8) & 0xFF)};
    pub_can_req_->publish(can_msg);
    RCLCPP_INFO(this->get_logger(), "Set heartbeat to %d ms", ms);
}

void KincoMotor::mode_req_cb(const std::shared_ptr<Int8> msg)
{
    VciCanObjMsg can_msg;

    can_msg.id = 0x600 + node_id_;
    can_msg.data_len = 5;
    can_msg.data = {0x2F, 0x60, 0x60, 0x00, static_cast<uint8_t>(msg->data & 0xFF)};
    pub_can_req_->publish(can_msg);
}

void KincoMotor::controlword_req_cb(const std::shared_ptr<UInt16> msg)
{
    VciCanObjMsg can_msg;

    can_msg.id = 0x600 + node_id_;
    can_msg.data_len = 6;
    can_msg.data = {0x2B, 0x40, 0x60, 0x00, static_cast<uint8_t>(msg->data & 0xFF), 
                                            static_cast<uint8_t>((msg->data >> 8) & 0xFF)};
    pub_can_req_->publish(can_msg);
}

void KincoMotor::rotate_dir_req_cb(const std::shared_ptr<UInt8> msg)
{
    VciCanObjMsg can_msg;

    can_msg.id = 0x600 + node_id_;
    can_msg.data_len = 5;
    can_msg.data = {0x2F, 0x7E, 0x60, 0x00, static_cast<uint8_t>(msg->data == 0 ? 0x0 : 0x01)};
    pub_can_req_->publish(can_msg);
}

void KincoMotor::speed_req_cb(const std::shared_ptr<UInt16> msg)
{
    uint32_t speed_DEC = (uint32_t) ((float) (msg->data) * 512.0 * 10000.0 / 1875.0);

    VciCanObjMsg can_msg;

    can_msg.id = 0x600 + node_id_;
    can_msg.data_len = 8;
    can_msg.data = {0x23, 0xFF, 0x60, 0x00, static_cast<uint8_t>(speed_DEC & 0xFF), 
                                            static_cast<uint8_t>((speed_DEC >> 8) & 0xFF), 
                                            static_cast<uint8_t>((speed_DEC >> 16) & 0xFF), 
                                            static_cast<uint8_t>((speed_DEC >> 24) & 0xFF)};
    pub_can_req_->publish(can_msg);
}

void KincoMotor::target_pos_req_cb(const std::shared_ptr<Int32> msg)
{
    VciCanObjMsg can_msg;

    can_msg.id = 0x600 + node_id_;
    can_msg.data_len = 8;
    can_msg.data = {0x23, 0x7A, 0x60, 0x00, static_cast<uint8_t>(msg->data & 0xFF), 
                                            static_cast<uint8_t>((msg->data >> 8) & 0xFF), 
                                            static_cast<uint8_t>((msg->data >> 16) & 0xFF), 
                                            static_cast<uint8_t>((msg->data >> 24) & 0xFF)};
    pub_can_req_->publish(can_msg);
}

void KincoMotor::profile_speed_req_cb(const std::shared_ptr<UInt32> msg)
{
    VciCanObjMsg can_msg;

    can_msg.id = 0x600 + node_id_;
    can_msg.data_len = 8;
    can_msg.data = {0x23, 0x81, 0x60, 0x00, static_cast<uint8_t>(msg->data & 0xFF), 
                                            static_cast<uint8_t>((msg->data >> 8) & 0xFF), 
                                            static_cast<uint8_t>((msg->data >> 16) & 0xFF), 
                                            static_cast<uint8_t>((msg->data >> 24) & 0xFF)};
    pub_can_req_->publish(can_msg);
}

void KincoMotor::pub_motor_state()
{
    const std::lock_guard<std::mutex> lock(this->mutex_);
    pub_motor_state_->publish(motor_state_);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<KincoMotor>(options));
  
  rclcpp::shutdown();
  return 0;
}