#include "can_transceiver/can_transceiver.hpp"

CanTransceiver::CanTransceiver(rclcpp::NodeOptions options)
    : Node("can_transceiver_node", options)
{
    if (!find_canusb())
    {
        RCLCPP_ERROR(this->get_logger(), "Find USBCAN Error!");
    }

    if (!open_canusb())
    {
        RCLCPP_ERROR(this->get_logger(), "Open USBCAN Error!");
    }

    if (!read_canusb())
    {
        RCLCPP_ERROR(this->get_logger(), "Read USBCAN Error!");
    }

    std::string _filename = "canusb_config.yaml";
    if (!init_canusb(_filename))
    {
        RCLCPP_ERROR(this->get_logger(), "Init USBCAN Error!");
        close_canusb();
        // TODO: exit ...
    }

    if (!start_canusb())
    {
        RCLCPP_ERROR(this->get_logger(), "Start USBCAN Error!");
        close_canusb();
        // TODO: exit ...
    }

    pub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::QoSInitialization can_qos_init = rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10);
    rclcpp::QoS can_qos(can_qos_init);
    can_qos.reliable();
    can_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    pub_can_msg_timer_ = this->create_wall_timer(5ms, std::bind(&CanTransceiver::can_msg_cb, this), pub_cb_group_);

    pub_can_msg_ = this->create_publisher<VciCanObjMsg>("can_msg", can_qos);

    sub_can_req_ = this->create_subscription<VciCanObjMsg>("can_req", can_qos, std::bind(&CanTransceiver::can_req_cb, this, _1));
}

bool CanTransceiver::find_canusb()
{
    int num = 0;

    num = VCI_FindUsbDevice2(pInfo1);
    RCLCPP_INFO(this->get_logger(), "USBCAN device num: %d PCS\n", num);
    if (num < 1)
        return false;

    for (int i = 0; i < num; i++)
	{
		RCLCPP_INFO(this->get_logger(), "Device: %d \n", i);
        RCLCPP_INFO(this->get_logger(), "Get VCI_ReadBoardInfo success!\n");
		
		RCLCPP_INFO(this->get_logger(), ">>Serial_Num: %s", pInfo1[i].str_Serial_Num);


		RCLCPP_INFO(this->get_logger(), "HW_Type: %s", pInfo1[i].str_hw_Type);

		RCLCPP_INFO(this->get_logger(), "Firmware Version: V%x.%x%x",
                                        (pInfo1[i].fw_Version & 0xF00) >> 8,
                                        (pInfo1[i].fw_Version & 0xF0)  >> 4,
                                        (pInfo1[i].fw_Version & 0xF));
	}

    return true;
}

bool CanTransceiver::open_canusb()
{
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1)
	{
		RCLCPP_INFO(this->get_logger(), "Open deivce success!");
	} 
    else
	{
		return false;
	}
    
    return true;
}

bool CanTransceiver::read_canusb()
{
    if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo) == 1)
	{
        RCLCPP_INFO(this->get_logger(), "Get VCI_ReadBoardInfo success!");
		
		RCLCPP_INFO(this->get_logger(), "Serial_Num: %s", pInfo.str_Serial_Num);

		RCLCPP_INFO(this->get_logger(), "HW_Type: %s", pInfo.str_hw_Type);

		RCLCPP_INFO(this->get_logger(), "Firmware Version: V%x.%x%x",
                                        (pInfo.fw_Version & 0xF00) >> 8,
                                        (pInfo.fw_Version & 0xF0)  >> 4,
                                        (pInfo.fw_Version & 0xF));
	}
    else
	{
		return false;
	}

    return true;
}

bool CanTransceiver::init_canusb(const std::string config_filename)
{
    std::string share_directory, config_file;
    try 
    {
        share_directory = ament_index_cpp::get_package_share_directory("can_transceiver");
        config_file = share_directory + "/config/" + config_filename;
        RCLCPP_INFO(this->get_logger(), "CANUSB initialization config file: %s", config_file.c_str());
    } 
    catch (ament_index_cpp::PackageNotFoundError & e) 
    {
        throw std::runtime_error(e.what());
    }

    if (config_file.empty())
        return false;

    try 
    {
        YAML::Node config_yaml = YAML::LoadFile(config_file);

        VCI_INIT_CONFIG config;

        config.AccCode = config_yaml["acc_code"].as<DWORD>();
        config.AccMask = config_yaml["acc_mask"].as<DWORD>();
        config.Filter = config_yaml["filter"].as<UCHAR>();
        config.Timing0 = config_yaml["timing0"].as<UCHAR>();
        config.Timing1 = config_yaml["timing1"].as<UCHAR>();
        config.Mode = config_yaml["mode"].as<UCHAR>();

        float baud_rate = 0.0;

        if (config.Timing0 == 0x18 && config.Timing1 == 0x1C) 
            baud_rate = 20;
        else if (config.Timing0 == 0x87 && config.Timing1 == 0xFF) 
            baud_rate = 40;
        else if (config.Timing0 == 0x09 && config.Timing1 == 0x1C) 
            baud_rate = 50;
        else if (config.Timing0 == 0x83 && config.Timing1 == 0xFF) 
            baud_rate = 80;
        else if (config.Timing0 == 0x04 && config.Timing1 == 0x1C) 
            baud_rate = 100;
        else if (config.Timing0 == 0x03 && config.Timing1 == 0x1C) 
            baud_rate = 125;
        else if (config.Timing0 == 0x81 && config.Timing1 == 0xFA) 
            baud_rate = 200;
        else if (config.Timing0 == 0x01 && config.Timing1 == 0x1C) 
            baud_rate = 250;
        else if (config.Timing0 == 0x80 && config.Timing1 == 0xFA) 
            baud_rate = 400;
        else if (config.Timing0 == 0x00 && config.Timing1 == 0x1C) 
            baud_rate = 500;
        else if (config.Timing0 == 0x80 && config.Timing1 == 0xB6) 
            baud_rate = 666;
        else if (config.Timing0 == 0x00 && config.Timing1 == 0x16) 
            baud_rate = 800;
        else if (config.Timing0 == 0x00 && config.Timing1 == 0x14) 
            baud_rate = 1000;
        else if (config.Timing0 == 0x09 && config.Timing1 == 0x6F) 
            baud_rate = 33.33;
        else if (config.Timing0 == 0x04 && config.Timing1 == 0x6F) 
            baud_rate = 66.66;
        else if (config.Timing0 == 0x03 && config.Timing1 == 0x6F) 
            baud_rate = 83.33;
        else
            return false;

        RCLCPP_INFO(this->get_logger(), "Baud rate: %.2f Kbps", baud_rate);
        
        if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Init CAN1 success");
        }
        else
        {
            return false;
        }
    }
    catch(const YAML::BadFile& e) 
    {
        throw std::runtime_error(e.what());
    } 
    catch(const YAML::ParserException& e) 
    {
        throw std::runtime_error(e.what());
    }

    return true;
}

bool CanTransceiver::start_canusb()
{
    if (VCI_StartCAN(VCI_USBCAN2, 0, 0) == 1)
	{
        RCLCPP_INFO(this->get_logger(), "Start CAN1 success");
	}
    else
    {
        return false;
    }

    return true;
}

bool CanTransceiver::close_canusb()
{
    VCI_CloseDevice(VCI_USBCAN2, 0);

    return true;
}

void CanTransceiver::can_msg_cb()
{
    const std::lock_guard<std::mutex> lock(this->mutex_);

    unsigned int rec_len = VCI_Receive(VCI_USBCAN2, 0, can_index, rec, rec_buf_len, 0);
    if (rec_len > 0)
    {
        for(unsigned int i = 0; i < rec_len; i++)
        {
            std::string extend, remote;
            if(rec[i].ExternFlag==0) extend="Standard";
            if(rec[i].ExternFlag==1) extend="Extend";
            if(rec[i].RemoteFlag==0) remote="Data";
            if(rec[i].RemoteFlag==1) remote="Remote";

            VciCanObjMsg msg;
            msg.id = rec[i].ID;
            msg.time_stamp = rec[i].TimeStamp;
            msg.time_flag = rec[i].TimeFlag;
            msg.send_type = rec[i].SendType;
            msg.remote_flag = rec[i].RemoteFlag;
            msg.extern_flag = rec[i].ExternFlag;
            msg.data_len = rec[i].DataLen;
            std::copy(std::begin(rec[i].Data), std::begin(rec[i].Data)+rec[i].DataLen, std::begin(msg.data));
            std::copy(std::begin(rec[i].Reserved), std::begin(rec[i].Reserved)+3, std::begin(msg.reserved));

            pub_can_msg_->publish(msg);

            RCLCPP_INFO(this->get_logger(), can_msg_format.c_str(),
                can_count++, can_index+1, "RX", rec[i].ID, extend.c_str(), remote.c_str(), rec[i].DataLen, 
                [&]() {
                    std::stringstream ss;
                    for (size_t j = 0; j < rec[i].DataLen; j++) {
                        ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(rec[i].Data[j]) << " ";
                    }
                    return ss.str();
                }().c_str());
        }
    }

}

void CanTransceiver::can_req_cb(const std::shared_ptr<VciCanObjMsg> msg)
{
    VCI_CAN_OBJ send[1];

	send[0].ID = msg->id;
	send[0].SendType = msg->send_type;
	send[0].RemoteFlag = msg->remote_flag;
	send[0].ExternFlag = msg->extern_flag;
	send[0].DataLen = msg->data_len;

    std::copy(std::begin(msg->data), std::end(msg->data), std::begin(send[0].Data));

    const std::lock_guard<std::mutex> lock(this->mutex_);
    if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
	{
        std::string extend, remote;
        if(msg->extern_flag==0) extend="Standard";
        if(msg->extern_flag==1) extend="Extend";
        if(msg->remote_flag==0) remote="Data";
        if(msg->remote_flag==1) remote="Remote";

        RCLCPP_INFO(this->get_logger(), can_msg_format.c_str(),
            can_count++, can_index+1, "TX", send[0].ID, extend.c_str(), remote.c_str(), send[0].DataLen, 
            [&]() {
                std::stringstream ss;
                for (size_t j = 0; j < send[0].DataLen; j++) {
                    ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(send[0].Data[j]) << " ";
                }
                return ss.str();
            }().c_str());
	}
    
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(CanTransceiver)

