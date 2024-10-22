///////////////////////////
// com.cpp

#include "md_robot_node/global.hpp"
#include "md_robot_node/main.hpp"
#include "md_robot_node/com.hpp"

#define MD_PROTOCOL_POS_PID             3
#define MD_PROTOCOL_POS_DATA_LEN        4
#define MD_PROTOCOL_POS_DATA_START      5

#define ENABLE_SERIAL_DEBUG             0

serial::Serial ser;

PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;
PID_IO_MONITOR_t curr_pid_io_monitor;
PID_ROBOT_MONITOR_t curr_pid_robot_monitor;
PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;

uint8_t serial_comm_rcv_buff[MAX_PACKET_SIZE];
uint8_t serial_comm_snd_buff[MAX_PACKET_SIZE];

extern uint32_t pid_response_receive_count;
extern uint32_t pid_request_cmd_vel_count;
extern INIT_SETTING_STATE_t fgInitsetting;
extern void MakeMDRobotMessage1(PID_PNT_MAIN_DATA_t *pData);
extern void MakeMDRobotMessage2(PID_ROBOT_MONITOR_t *pData);
extern void PubRobotOdomMessage(void);
extern void PubRobotRPMMessage(void);

extern std::string serial_port;

//Initialize serial communication in ROS
int InitSerialComm(void)
{
    std::string port;

    port = "/dev/" + serial_port;
    ROS_INFO("Serial port: %s", port.c_str());
    ROS_INFO("Serial baudrate: %d", robotParamData.nBaudrate);

    try
    {
        ser.setPort(port);
        ser.setBaudrate(robotParamData.nBaudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1667); //1667 when baud is 57600, 0.6ms
        ser.setTimeout(to);                                        //2857 when baud is 115200, 0.35ms
        ser.open();
    }

    catch (serial::IOException& e)
    {
        printf("\r\n");
        ROS_ERROR_STREAM("Unable to open port\r\n");
        return -1;
    }

    if(ser.isOpen()) {
        printf("\r\n");
        ROS_INFO_STREAM("*Serial port open succes!\r\n");
        return 1;
    }
    else {
        return -1;
    }
}

uint8_t CalCheckSum(uint8_t *pData, uint16_t length)
{
    uint16_t sum;

    sum = 0;
    for(int i = 0; i < length; i++) {
        sum += *pData++;
    }

    sum = ~sum + 1; //check sum

    return sum;
}

int PutMdData(PID_CMD_t pid, uint16_t rmid, const uint8_t *pData, uint16_t length)
{
    uint8_t *p;
    uint16_t len;

    len = 0;
    serial_comm_snd_buff[len++] = rmid;
    serial_comm_snd_buff[len++] = robotParamData.nIDPC;
    serial_comm_snd_buff[len++] = 1;
    serial_comm_snd_buff[len++] = (uint8_t)pid;
    serial_comm_snd_buff[len++] = length;

    p = (uint8_t *)&serial_comm_snd_buff[len];
    memcpy((char *)p, (char *)pData, length);
    len += length;

    serial_comm_snd_buff[len++] = CalCheckSum(serial_comm_snd_buff, len);

    if(ser.isOpen() == true) {
        ser.write(serial_comm_snd_buff, len);
    }

    return 1;
}

int MdReceiveProc(void) //save the identified serial data to defined variable according to PID NUMBER data
{
    uint8_t *pRcvBuf;
    uint8_t *pRcvData;
    uint8_t byRcvPID;
    uint8_t byRcvDataSize;

    pRcvBuf = serial_comm_rcv_buff;

    byRcvPID      = pRcvBuf[MD_PROTOCOL_POS_PID];
    byRcvDataSize = pRcvBuf[MD_PROTOCOL_POS_DATA_LEN];
    pRcvData      = &pRcvBuf[MD_PROTOCOL_POS_DATA_START];

    switch(byRcvPID)
    {
        case PID_IO_MONITOR:                // 194
        {
            ROS_INFO("RCV: PID_IO_MONITOR: %d, %d", byRcvDataSize, (int)sizeof(PID_IO_MONITOR_t));
            memcpy((char *)&curr_pid_io_monitor, (char *)pRcvData, sizeof(PID_PNT_MAIN_DATA_t));
            ROS_INFO("Inout voltage: %d", curr_pid_io_monitor.input_voltage);
            break;
        }

        case PID_ROBOT_MONITOR2:
        {
            ROS_INFO("RCV: PID_ROBOT_MONITOR2: %d, %d", byRcvDataSize, (int)sizeof(PID_ROBOT_MONITOR2_t));
            memcpy((char *)&curr_pid_robot_monitor2, (char *)pRcvData, sizeof(PID_ROBOT_MONITOR2_t));
            ROS_INFO("Inout voltage: %d", curr_pid_robot_monitor2.sVoltIn);
            break;
        }

        case PID_PNT_MAIN_DATA: // 210
        {
            if(byRcvDataSize == sizeof(PID_PNT_MAIN_DATA_t)) {
                ROS_INFO("RCV: PID_PNT_MAIN_DATA: %d, %d", byRcvDataSize, (int)sizeof(PID_PNT_MAIN_DATA_t));

                pid_response_receive_count++;
                pid_request_cmd_vel_count = 2;

                memcpy((char *)&curr_pid_pnt_main_data, (char *)pRcvData, sizeof(PID_PNT_MAIN_DATA_t));

                MakeMDRobotMessage1(&curr_pid_pnt_main_data);

                PubRobotRPMMessage();
            }
            break;
        }

        case PID_ROBOT_MONITOR:        // 253
        {
            if(byRcvDataSize == sizeof(PID_ROBOT_MONITOR_t)) {
                // ROS_INFO("RCV: PID_ROBOT_MONITOR_t: %d, %d", byRcvDataSize, (int)sizeof(PID_ROBOT_MONITOR_t));

                pid_response_receive_count++;
                pid_request_cmd_vel_count = 2;

                memcpy((char *)&curr_pid_robot_monitor, (char *)pRcvData, sizeof(PID_ROBOT_MONITOR_t));
#if 1
                ROS_INFO("%d", curr_pid_robot_monitor.lTempPosi_x);
                ROS_INFO("%d", curr_pid_robot_monitor.lTempPosi_y);
                ROS_INFO("%d", curr_pid_robot_monitor.sTempTheta);
                ROS_INFO("%d", curr_pid_robot_monitor.battery_percent);
                ROS_INFO("%d", curr_pid_robot_monitor.byUS1);
                ROS_INFO("%d", curr_pid_robot_monitor.byUS2);
                ROS_INFO("%d", curr_pid_robot_monitor.byUS3);
                ROS_INFO("%d", curr_pid_robot_monitor.byUS4);
                ROS_INFO("0x%x", (uint8_t)curr_pid_robot_monitor.byPlatStatus.val);
                ROS_INFO("%d", curr_pid_robot_monitor.linear_velocity);
                ROS_INFO("%d", curr_pid_robot_monitor.angular_velocity);
                ROS_INFO(" ");
#endif                

                if(robotParamData.use_MDUI == 1) {  // If using MDUI
                    MakeMDRobotMessage2(&curr_pid_robot_monitor);

                    PubRobotOdomMessage();
                }
            }
            break;
        }

        case PID_ROBOT_PARAM:  // 247
        {
            // ROS_INFO("RCV: PID_ROBOT_PARAM: %d, %d", byRcvDataSize, (int)sizeof(PID_ROBOT_PARAM_t));
            if(byRcvDataSize == sizeof(PID_ROBOT_PARAM_t)) {
                PID_ROBOT_PARAM_t *p;

                p = (PID_ROBOT_PARAM_t *)pRcvData;
#if 0
                ROS_INFO("Diameter: %d", p->nDiameter);
                ROS_INFO("Wheel Length: %d", p->nWheelLength);
                ROS_INFO("Gear Ratio: %d", p->nGearRatio);
#endif                
            }
            break;
        }

        default:
            break;
    }
    return 1;
}

int AnalyzeReceivedData(uint8_t byArray[], uint8_t byBufNum) //Analyze the communication data
{
    ros::NodeHandle n;
    ros::Time stamp;
    uint8_t i, j;
    uint8_t data;
    static uint8_t byChkSec;
    static long lExStampSec, lExStampNsec;
    static uint32_t byPacketNum;
    static uint32_t rcv_step;
    static uint8_t byChkSum;
    static uint16_t byMaxDataNum;
    static uint16_t byDataNum;

    if(byPacketNum >= MAX_PACKET_SIZE)
    {
        rcv_step = 0;
        byPacketNum = 0;

        return 0;
    }
    
    for(j = 0; j < byBufNum; j++)
    {
        data = byArray[j];
#if (ENABLE_SERIAL_DEBUG == 1)
        printf("%02x(%3d) ", data, data);
#endif
        switch(rcv_step) {
            case 0:    //Put the reading machin id after checking the data
                if(data == robotParamData.nIDPC)
                {
                    byPacketNum = 0;
                    byChkSum = data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    byPacketNum = 0;
#if (ENABLE_SERIAL_DEBUG == 1)                    
                    ROS_INFO("received ID: %d(0x%02x)", data, data);
                    ROS_INFO("error.ser: %s, %d", __FILE__, __LINE__);
#endif
                }
                break;

            case 1:    //Put the transmitting machin id after checking the data
                if((data == robotParamData.nIDMDUI) || (data == robotParamData.nIDMDT))
                {
                    byChkSum += data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    rcv_step = 0;
                    byPacketNum = 0;
#if (ENABLE_SERIAL_DEBUG == 1)                    
                    ROS_INFO("error.ser: %s, %d", __FILE__, __LINE__);
#endif
                }
                break;

            case 2:    //Check ID
                if(data == 1 || data == ID_ALL)
                {
                    byChkSum += data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    rcv_step = 0;
                    byPacketNum = 0;
#if (ENABLE_SERIAL_DEBUG == 1)                    
                    ROS_INFO("error.ser: %s, %d", __FILE__, __LINE__);
#endif
                }
                break;

             case 3:    //Put the PID number into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                rcv_step++;
                break;

             case 4:    //Put the DATANUM into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                byMaxDataNum = data;
                byDataNum = 0;

                rcv_step++;
                break;

             case 5:    //Put the DATA into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                if(++byDataNum >= MAX_DATA_SIZE)
                {
                    rcv_step = 0;
#if (ENABLE_SERIAL_DEBUG == 1)                    
                    ROS_INFO("error.ser: %s, %d", __FILE__, __LINE__);
#endif
                    break;
                }

                if(byDataNum >= byMaxDataNum) {
                    rcv_step++;
                }
                break;

             case 6:    //Put the check sum after Checking checksum
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                if(byChkSum == 0)
                {
#if (ENABLE_SERIAL_DEBUG == 1)
                    printf("\r\n");
#endif
                    MdReceiveProc();                                 //save the identified serial data to defined variable
                }
                else {
                    ROS_INFO("Error.Checksum");
                }

                byPacketNum = 0;

                rcv_step = 0;
                break;

            default:
                rcv_step = 0;
                break;
        }
    }
    return 1;
}

int ReceiveSerialData(void) //Analyze the communication data
{
    uint8_t byRcvBuf[250];
    uint8_t byBufNumber;

    static uint8_t tempBuffer[250];
    static uint8_t tempLength;

    byBufNumber = ser.available();
    if(byBufNumber != 0)
    {
        if(byBufNumber > sizeof(byRcvBuf)) {
            byBufNumber = sizeof(byRcvBuf);
        }

        ser.read(byRcvBuf, byBufNumber);

        memcpy(tempBuffer, byRcvBuf, byBufNumber);
        tempLength = byBufNumber;

        AnalyzeReceivedData(tempBuffer, tempLength);
    }

    return 1;
}

/////////////// the end of file