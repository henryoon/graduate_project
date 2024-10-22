///////////////////////
// main.cpp
// Publisher, Subscrive action added
#include <geometry_msgs/Twist.h>

#include "md_robot_node/global.hpp"
#include "md_robot_node/main.hpp"
#include "md_robot_node/com.hpp"
#include "md/md_robot_msg1.h"
#include "md/md_robot_msg2.h"
#include <ros/ros.h>
#include <queue>

#define MAX_CONNECTION_CHECK_COUNT              10

ros::Publisher md_robot_message1_pub;
md::md_robot_msg1 md_robot_msg_pid_pnt_main_data;

ros::Publisher md_robot_message2_pub;
md::md_robot_msg2 md_robot_msg_pid_robot_monitor;

ROBOT_PARAMETER_t robotParamData;

SETTINNG_PARAM_STEP_t byCntInitStep;
uint16_t byCntComStep;
uint32_t velCmdUpdateCount;
uint32_t velCmdRcvCount;
uint32_t pid_response_receive_count;
uint32_t pid_request_cmd_vel_count;
volatile bool mdui_mdt_connection_state;
volatile bool remote_pc_connection_state;

INIT_SETTING_STATE_t fgInitsetting;
uint16_t check_connection_retry_count;

double goal_cmd_speed;             // m/sec
double goal_cmd_ang_speed;         // radian/sec
bool reset_pos_flag;
bool reset_alarm_flag;

extern PID_ROBOT_MONITOR_t curr_pid_robot_monitor;
extern PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;

extern int InitSerialComm(void);
extern int16_t * RobotSpeedToRPMSpeed(double linear, double angular);

// callback function: 1sec period
void VelCmdRcvTimeoutCallback(const ros:: TimerEvent&)
{
    static uint32_t old_velCmdRcvCount;
    static uint32_t old_pid_response_receive_count;

    if(velCmdRcvCount == old_velCmdRcvCount) {
        goal_cmd_speed = 0;
        goal_cmd_ang_speed = 0;

        if(remote_pc_connection_state == true) {
            velCmdUpdateCount++;

            // ROS_INFO("Error.cmd_vel topic --> interface error");
            // PC vs Remote PC --> interface error
            remote_pc_connection_state = false;
        }
    }
    else {
        old_velCmdRcvCount = velCmdRcvCount;

        if(remote_pc_connection_state == false) {
            // ROS_INFO("Ok.cmd_vel topic --> interface OK");
            remote_pc_connection_state = true;
        }
    }

    if(pid_request_cmd_vel_count > 5) {
        if(mdui_mdt_connection_state == true) {
            // ROS_INFO("Error.RS232 or RS485 --> nterface error");
            // Remote PC vs MDUI or MDT --> nterface error
            mdui_mdt_connection_state = false;
        }
    }
    else if(pid_request_cmd_vel_count == 2) {
        if(mdui_mdt_connection_state == false) {
            // ROS_INFO("Ok.RS232 or RS485 --> Ok");
            mdui_mdt_connection_state = true;
        }
    }
}

void InitMotorParameter(void)               // If using MDUI
{
    switch(byCntInitStep)
    {
        case SETTING_PARAM_STEP_PID_PNT_VEL_CMD:
        {
            PID_PNT_VEL_CMD_t cmd_data, *p;

#if 1
            ROS_INFO("[SET] PID_PNT_VEL_CMD(PID NO: %d)", PID_PNT_VEL_CMD);
            // ROS_INFO("size of PID_PNT_VEL_CMD_t: %ld", sizeof(PID_PNT_VEL_CMD_t));
#endif

            p = &cmd_data;
            p->enable_id1 = 1;
            p->rpm_id1 = 0;
            p->enable_id2 = 1;
            p->rpm_id2 = 0;

            if(robotParamData.use_MDUI == 1) {              // Use only if you use MDUI
                p->req_monitor_id = REQUEST_PID_ROBOT_MONITOR;
            }
            else {
                p->req_monitor_id = REQUEST_PNT_MAIN_DATA;
            }

            pid_response_receive_count = 0;
            pid_request_cmd_vel_count = 1;
            PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t *)&cmd_data, sizeof(cmd_data));

            byCntInitStep = SETTING_PARAM_WAIT;
            break;
        }

        case SETTING_PARAM_WAIT:
        {
            if(pid_response_receive_count > 0) {
                pid_response_receive_count = 0;
                byCntInitStep = SETTING_PARAM_STEP_PID_ROBOT_PARAM;
            }
            else {
                check_connection_retry_count++;
                if(check_connection_retry_count >= MAX_CONNECTION_CHECK_COUNT) {
                    ROS_INFO("!!! Error RS232(MDUI) or RS485(MDT) !!!");
                    fgInitsetting = INIT_SETTING_STATE_ERROR;
                    byCntInitStep = SETTING_PARAM_STEP_DONE;
                }
                else {
                    byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
                }
            }
            break;
        }

        case SETTING_PARAM_STEP_PID_ROBOT_PARAM:
        {
            if(robotParamData.use_MDUI == 1) {              // Use only if you use MDUI
                PID_ROBOT_PARAM_t cmd_data, *p;

#if 1
                ROS_INFO("[SET] SETTING_PARAM_STEP_PID_ROBOT_PARAM(PID NO: %d)", SETTING_PARAM_STEP_PID_ROBOT_PARAM);
                // ROS_INFO("size of PID_ROBOT_PARAM_t: %ld", sizeof(c));
#endif
                p = &cmd_data;
                p->nDiameter = (uint16_t)robotParamData.nDiameter;
                p->nWheelLength = (uint16_t)(robotParamData.nWheelLength * 1000.0);                 // m unit --> mm unit
                p->nGearRatio = (uint16_t)robotParamData.nGearRatio;

                ROS_INFO("D: %d, L: %d, R: %d", p->nDiameter, p->nWheelLength, p->nGearRatio);
                
                PutMdData(PID_ROBOT_PARAM, MID_MDUI, (const uint8_t *)p, sizeof(cmd_data));     // 247
            }

            byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
            break;
        }

        case SETTING_PARAM_STEP_PID_POSI_RESET:
        {
            uint8_t dummy;

#if 1
            ROS_INFO("[SET] PID_POSI_RESET(PID NO: %d)", PID_POSI_RESET);
#endif

            dummy = 0;
            PutMdData(PID_POSI_RESET, robotParamData.nRMID, (const uint8_t *)&dummy, sizeof(dummy));

            byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_START;
            break;
        }

        case SETTING_PARAM_STEP_PID_SLOW_START:
        {
            PID_SLOW_START_t cmd_data, *p;

#if 1
            ROS_INFO("[SET] PID_SLOW_START(PID NO: %d)", PID_SLOW_START);
#endif

            p = &cmd_data;
            p->value = robotParamData.nSlowstart;

            PutMdData(PID_SLOW_START, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

            byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_DOWN;
            break;
        }

        case SETTING_PARAM_STEP_PID_SLOW_DOWN:
        {
            PID_SLOW_DOWN_t cmd_data, *p;

#if 1
            ROS_INFO("[SET] PID_SLOW_DOWN(PID NO: %d)", PID_SLOW_DOWN);
#endif

            p = &cmd_data;
            p->value = robotParamData.nSlowdown;

            PutMdData(PID_SLOW_DOWN, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

            byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD;
            break;
        }

        case SETTING_PARAM_STEP_PID_INV_SIGH_CMD:       // Left motor direction
        {
            uint8_t cmd_data;

#if 1
            ROS_INFO("[SET] PID_INV_SIGN_CMD(PID NO: %d)", PID_INV_SIGN_CMD);
#endif

            if(robotParamData.reverse_direction == 0) {
                cmd_data = 1;
            }
            else {
                cmd_data = 0;
            }

            PutMdData(PID_INV_SIGN_CMD, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

            byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD2;
            break;
        }

        case SETTING_PARAM_STEP_PID_INV_SIGH_CMD2:      // Right motor direction
        {
            uint8_t cmd_data;

#if 1
            ROS_INFO("[SET] PID_INV_SIGN_CMD2(PID NO: %d)", PID_INV_SIGN_CMD2);
#endif

            if(robotParamData.reverse_direction == 0) {
                cmd_data = 0;
            }
            else {
                cmd_data = 1;
            }

            PutMdData(PID_INV_SIGN_CMD2, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

            byCntInitStep = SETTING_PARAM_STEP_PID_USE_EPOSI;
            break;
        }

        case SETTING_PARAM_STEP_PID_USE_EPOSI:
        {
            uint8_t cmd_data;

#if 1
            ROS_INFO("[SET] PID_USE_POSI(PID NO: %d)", PID_USE_POSI);
#endif

            // Encoder function: Use the encoder only when using an in-wheel motor
            if(robotParamData.enable_encoder == 0) {                // disable encoder
                cmd_data = 0;               // hall sensor
            }
            else {                                                  // enable encoder
                cmd_data = 1;               // encoder
            }

            PutMdData(PID_USE_POSI, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

            byCntInitStep = SETTING_PARAM_STEP_PID_PPR;
            break;
        }

        case SETTING_PARAM_STEP_PID_PPR:
        {
            // Use the encoder only when using an in-wheel motor
            if(robotParamData.use_MDUI == 1 && robotParamData.enable_encoder == 1) {              // Use only if you use MDUI
                PID_PPR_t cmd_data, *p;

#if 1
                ROS_INFO("[SET] PID_PPR(PID NO: %d)", PID_PPR);
#endif
                p = &cmd_data;

                p->PPR = robotParamData.encoder_PPR;

                PutMdData(PID_PPR, robotParamData.nRMID, (const uint8_t *)&cmd_data, sizeof(PID_PPR_t));
            }

            byCntInitStep = SETTING_PARAM_STEP_DONE;

            if(pid_request_cmd_vel_count == 2) {
                fgInitsetting = INIT_SETTING_STATE_OK;
            }
            else {
                fgInitsetting = INIT_SETTING_STATE_ERROR;

                if(robotParamData.use_MDUI == 1) {
                    ROS_INFO("!!! Error RS232 interface !!!");
                }
                else {
                    ROS_INFO("!!! Error RS485 interface !!!");
                }
            }
            break;
        }

        default:
            break;
    }
}

void RequestRobotStatus(void)
{
    int nArray[5];
    uint8_t req_pid;
    int16_t *pGoalRPMSpeed;

    switch(byCntComStep)
    {
        case 0:
        {
            // ROS_INFO("REQ: PID_PNT_VEL_CMD & PID_PNT_MAIN_DATA");
            if(velCmdUpdateCount > 0) 
            {
                velCmdUpdateCount = 0;

                PID_PNT_VEL_CMD_t pid_pnt_vel_cmd, *p;

                if(mdui_mdt_connection_state == true) {
                    pGoalRPMSpeed = RobotSpeedToRPMSpeed(goal_cmd_speed, goal_cmd_ang_speed);
                }
                else {
                    pGoalRPMSpeed[0] = 0;
                    pGoalRPMSpeed[1] = 0;
                }

#if 1
                ROS_INFO("Goal %.2f, %.2f, RPM L:%d, R:%d", goal_cmd_speed, goal_cmd_ang_speed, pGoalRPMSpeed[0], pGoalRPMSpeed[1]);
#endif

                p = &pid_pnt_vel_cmd;
                p->enable_id1 = 1;
                p->rpm_id1 = pGoalRPMSpeed[0];
                p->enable_id2 = 1;
                p->rpm_id2 = pGoalRPMSpeed[1];

                if(robotParamData.use_MDUI == 1) {              // Use only if you use MDUI
                    p->req_monitor_id = REQUEST_PID_ROBOT_MONITOR;
                }
                else {
                    p->req_monitor_id = REQUEST_PNT_MAIN_DATA;
                }

                PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t *)&pid_pnt_vel_cmd, sizeof(pid_pnt_vel_cmd));

                pid_request_cmd_vel_count++;
            }

            if(robotParamData.use_MDUI == 1) {              // Use only if you use MDUI
                if(curr_pid_robot_monitor.byPlatStatus.bits.bEmerSW == 1)
                {
                    byCntComStep = 3;
                    break;
                }
            }

            if(reset_pos_flag == true || reset_alarm_flag == true) {
                byCntComStep = 3;
                break;
            }
            else {
                byCntComStep = 4;
                break;
            }
            break;
        }

        case 3:
        {
            if(robotParamData.use_MDUI == 1) {              // Use only if you use MDUI
                if(curr_pid_robot_monitor.byPlatStatus.bits.bEmerSW == 1)
                {
                    PID_PNT_TQ_OFF_t pid_pnt_tq_off, *p;

                    pid_pnt_tq_off.enable_id1 = 1;
                    pid_pnt_tq_off.enable_id2 = 1;
                    pid_pnt_tq_off.req_monitor_id = REQUEST_PNT_MAIN_DATA;
                    PutMdData(PID_PNT_TQ_OFF, robotParamData.nRMID, (const uint8_t *)&pid_pnt_tq_off, sizeof(pid_pnt_tq_off));
                }
            }
            
            if(reset_pos_flag == true) {
                uint8_t dummy;  
                
                reset_pos_flag = false;

                dummy = 0;
                PutMdData(PID_POSI_RESET, robotParamData.nRMID, (const uint8_t *)&dummy, sizeof(dummy));
            }
            else if(reset_alarm_flag == true) {
                uint8_t cmd_pid;

                reset_alarm_flag = false;

                cmd_pid = CMD_ALARM_RESET;
                PutMdData(PID_COMMAND, robotParamData.nRMID, (const uint8_t *)&cmd_pid, 1);
            }

            byCntComStep = 0;
            break;
        }

        case 4:
        {
            uint8_t request_pid;  

            if(robotParamData.use_MDUI == 0) { 
                request_pid = PID_IO_MONITOR;

                ROS_INFO("REQ: PID_IO_MONITOR");

                PutMdData(PID_REQ_PID_DATA, robotParamData.nRMID, (const uint8_t *)&request_pid, 1);
            }
            else {
                request_pid = PID_ROBOT_MONITOR2;

                ROS_INFO("REQ: PID_ROBOT_MONITOR2");

                PutMdData(PID_REQ_PID_DATA, robotParamData.nRMID, (const uint8_t *)&request_pid, 1);
            }

            byCntComStep = 0;
            break;
        }

        default:
            byCntComStep = 0;
            break;
    }
}

void cmdVelCallBack(const geometry_msgs::Twist& keyVel)   //from turtlebot3_teleop_key node
{
    if(fgInitsetting == INIT_SETTING_STATE_OK) {
        velCmdRcvCount++;
        velCmdUpdateCount++;

        goal_cmd_speed = keyVel.linear.x;
        goal_cmd_ang_speed = keyVel.angular.z;

#if 0
        ROS_INFO("Goal cmd_vel(m/sec): lin: %f, ang: %f", keyVel.linear.x, keyVel.angular.z);
#endif
    }

    return;
}

void resetPositionCallBack(const std_msgs::Bool& reset_position_msg)
{
    if(reset_position_msg.data == 1) {
        ROS_INFO("Reset Position");
        reset_pos_flag = true;
    }
}

void resetAlarmCallBack(const std_msgs::Bool& reset_alarm_msg)
{
    if(reset_alarm_msg.data == 1) {
        ROS_INFO("Reset Alarm");
        reset_alarm_flag = true;
    }
}

std::string serial_port;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "md"); //Node name initialization.
    ros::NodeHandle nh;        //Node handle declaration for communication with ROS system.

    ros::Subscriber keyboard_sub = nh.subscribe("cmd_vel", 10, cmdVelCallBack);
    ros::Subscriber reset_position_sub = nh.subscribe("reset_position", 10, resetPositionCallBack);              //Subscriber declaration.
    ros::Subscriber reset_alarm_sub = nh.subscribe("reset_alarm", 10, resetAlarmCallBack);              //Subscriber declaration.
    int16_t *pGoalRPMSpeed;
    uint32_t serial_baudrate; 

    reset_pos_flag = false;
    reset_alarm_flag = false;

    fgInitsetting = INIT_SETTING_STATE_NONE;

    nh.getParam("md_robot_node/use_MDUI", robotParamData.use_MDUI);

    robotParamData.nIDPC = MID_REMOTE_PC;         // Platform mini-PC ID
    robotParamData.nIDMDUI = MID_MDUI;       // MDUI ID
    robotParamData.nIDMDT = MID_MDT;        // MD750T, MD400T, MD200T ID

    ROS_INFO("----------------------------------");
    ROS_INFO(" Version 2.6");
    ROS_INFO(" 2024.07.11");
    ROS_INFO("----------------------------------");

    if(robotParamData.use_MDUI == 1) {  // If using MDUI
        robotParamData.nRMID = robotParamData.nIDMDUI;
        ROS_INFO("----------------------------------");
        ROS_INFO(" Using MDUI(RS232)");
        ROS_INFO("----------------------------------");
    }
    else {
        robotParamData.nRMID = robotParamData.nIDMDT;
        ROS_INFO("----------------------------------");
        ROS_INFO(" Direct MDT(RS485)");
        ROS_INFO("----------------------------------");
    }
    
    nh.getParam("md_robot_node/serial_port", serial_port);
    nh.getParam("md_robot_node/serial_baudrate", robotParamData.nBaudrate);

    nh.getParam("md_robot_node/reverse_direction", robotParamData.reverse_direction);
    nh.getParam("md_robot_node/maxrpm", robotParamData.nMaxRPM);
    nh.getParam("md_robot_node/enable_encoder", robotParamData.enable_encoder);
    nh.getParam("md_robot_node/slow_start", robotParamData.nSlowstart);
    nh.getParam("md_robot_node/slow_down", robotParamData.nSlowdown);
    nh.getParam("md_robot_node/wheel_length", robotParamData.nWheelLength);                // m unit
    nh.getParam("md_robot_node/reduction", robotParamData.nGearRatio);
    nh.getParam("md_robot_node/wheel_radius", robotParamData.wheel_radius);               // m unit
    nh.getParam("md_robot_node/encoder_PPR", robotParamData.encoder_PPR);
    
    robotParamData.nDiameter = (int)(robotParamData.wheel_radius * 2.0 * 1000.0);              // nDiameter is (mm) unit

    md_robot_message1_pub = nh.advertise<md::md_robot_msg1>("md_robot_message1", 10);
    md_robot_message2_pub = nh.advertise<md::md_robot_msg2>("md_robot_message2", 10);

    ROS_INFO("Serial port             : %s", serial_port.c_str());
    ROS_INFO("Baudrate                : %d bps", robotParamData.nBaudrate);
    ROS_INFO("Diameter(mm)            : %d", robotParamData.nDiameter);
    ROS_INFO("Wheel Radius(m)         : %f", robotParamData.wheel_radius);
    ROS_INFO("WheelLength(m)          : %f", robotParamData.nWheelLength);
    ROS_INFO("Reduction rate          : %d", robotParamData.nGearRatio);

    ROS_INFO("Max RPM                 : %d", robotParamData.nMaxRPM);
    if(robotParamData.reverse_direction == 0) {
        ROS_INFO("Robot direction         : Forward");
    }
    else {
        ROS_INFO("Robot direction         : Reverse");
    }

    if(robotParamData.enable_encoder == 0) {                // disable encoder
        ROS_INFO("motor position detection: hall sensor");
    }
    else {                                                  // enable encoder
        ROS_INFO("motor position detection: encoder");

        if(robotParamData.use_MDUI == 1) {  // If using MDUI
            // If use hall sensor: 3 x pole no x reduction rate
            // If use encoder: 4 x encder x reduction rate
            ROS_INFO(" PPR                    : %d", robotParamData.encoder_PPR);
        }
    }

    ROS_INFO("Slow start              : %d", robotParamData.nSlowstart);
    ROS_INFO("Slow down               : %d", robotParamData.nSlowdown);

    if(InitSerialComm() == -1) {     //communication initialization in com.cpp 
        return 1;
    }

    ros::Rate init_loop_rate(10);
    ros::Time start_time = ros::Time::now();
    ros::Duration start_delay(0.5);
    double start_delay_sec = ros::Time::now().toSec();

    start_delay_sec += start_delay.toSec();

    //---------------------------------------------------------------------------------------------------------
    // Start delay: 0.5sec
    //---------------------------------------------------------------------------------------------------------
    while(ros::ok())
    {
        if(ros::Time::now().toSec() >= start_delay_sec) {
            break;
        }

        ReceiveSerialData();

        ros::spinOnce();
        init_loop_rate.sleep();
    }

    byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
    check_connection_retry_count = 0;
    fgInitsetting = INIT_SETTING_STATE_NONE;

    while(ros::ok())
    {
        ReceiveSerialData();

        InitMotorParameter();

        if(fgInitsetting != INIT_SETTING_STATE_NONE) {
            break;
        }

        ros::spinOnce();
        init_loop_rate.sleep();
    }

    if(fgInitsetting == INIT_SETTING_STATE_OK) {
        ROS_INFO("[Init done]\r\n");

        mdui_mdt_connection_state = true;
        remote_pc_connection_state = false;
    }
    else {
        ROS_INFO("error.init ROBOT");

        return 1;
    }

    pid_request_cmd_vel_count = 0;
    byCntComStep = 0;

    ros::Rate request_loop_rate(10);       // 10Hz
    ros::Timer vel_cmd_rcv_timeout = nh.createTimer(ros::Duration(1.0), VelCmdRcvTimeoutCallback);
    vel_cmd_rcv_timeout.start();

    static bool old_mdui_mdt_connection_state = false;
    static bool old_remote_pc_connection_state = false;

    while(ros::ok())
    {
        if(remote_pc_connection_state == false && old_remote_pc_connection_state == true) {
            ROS_INFO("Remote PC connection state: error!!!\r\n");
            old_remote_pc_connection_state = remote_pc_connection_state;
            pid_request_cmd_vel_count = 0;
        }
        else if(remote_pc_connection_state == true && old_remote_pc_connection_state == false) {
            ROS_INFO("Remote PC connection state: Ok\r\n");
            old_remote_pc_connection_state = remote_pc_connection_state;
            pid_request_cmd_vel_count = 0;
        }

        if(mdui_mdt_connection_state == false && old_mdui_mdt_connection_state == true) {
            if(robotParamData.use_MDUI == 1) {
                ROS_INFO("MDUI connection state: error!\r\n");
            }
            else {
                ROS_INFO("MDT connection state: error!\r\n");
            }
            old_mdui_mdt_connection_state = mdui_mdt_connection_state;
        }
        else if(mdui_mdt_connection_state == true && old_mdui_mdt_connection_state == false) {
            if(robotParamData.use_MDUI == 1) {
                ROS_INFO("MDUI connection state: Ok\r\n");
            }
            else {
                ROS_INFO("MDT connection state: Ok\r\n");
            }
            old_mdui_mdt_connection_state = mdui_mdt_connection_state;
        }

        ReceiveSerialData();

        RequestRobotStatus();

        ros::spinOnce();
        request_loop_rate.sleep();
    }
}

void PubRobotRPMMessage(void)               // This is the message used by default
{
    md_robot_message1_pub.publish(md_robot_msg_pid_pnt_main_data);
}

void PubRobotOdomMessage(void)             // Use only when using MDUI
{
    md_robot_message2_pub.publish(md_robot_msg_pid_robot_monitor);
}
