#include <stdint.h>
#include "md_robot_node/global.hpp"
#include "md_robot_node/main.hpp"
#include "md_robot_node/com.hpp"
#include "md/md_robot_msg1.h"
#include "md/md_robot_msg2.h"

#define ENABLE_MD_MESSAGE          

#define VELOCITY_CONSTANT_VALUE         9.5492743       // linear speed(m/min), v = the length of circle of wheel x RPM
                                                        // linear speed(m/sec), v = (2 x wheel radius x (pi / 60) x RPM)
                                                        // 0.10472 = (2 x pi / 60)
                                                        // V = r * w = r * (RPM * 0.10472)
                                                        //           = r * RPM * 0.10472
                                                        // RPM = V / r * 9.5492743

#define constrain(amt,low,high) ((amt)<=(low)?(low):((amt)>=(high)?(high):(amt)))

#define LEFT           	  0      // Swing direction
#define RIGHT             1

extern md::md_robot_msg1 md_robot_msg_pid_pnt_main_data;
extern md::md_robot_msg2 md_robot_msg_pid_robot_monitor;
extern PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;
extern PID_IO_MONITOR_t curr_pid_io_monitor;

// m/sec --> RPM
int16_t * RobotSpeedToRPMSpeed(double linear, double angular)
{
    double wheel_radius;
    double wheel_separation;
    double reduction;
    double wheel_velocity_cmd[2];
    static int16_t goal_rpm_spped[2];

    wheel_radius = robotParamData.wheel_radius;
    wheel_separation = robotParamData.nWheelLength;
    reduction = (double)robotParamData.nGearRatio;

    // ROS_INFO("l:%f, a:%f", (double)linear, (double)angular);

    wheel_velocity_cmd[LEFT]   = linear - (angular * wheel_separation / 2);
    wheel_velocity_cmd[RIGHT]  = linear + (angular * wheel_separation / 2);

    // ROS_INFO("left:%f, right:%f", (double)wheel_velocity_cmd[LEFT], (double)wheel_velocity_cmd[RIGHT]);

    //***************************************************************************************
    // Convert the linearvelocity to RPM 
    //***************************************************************************************
    wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -robotParamData.nMaxRPM, robotParamData.nMaxRPM);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -robotParamData.nMaxRPM, robotParamData.nMaxRPM);

    // ROS_INFO("RPM1 L:%f, R:%f\r\n", (double)wheel_velocity_cmd[LEFT], (double)wheel_velocity_cmd[RIGHT]);

    goal_rpm_spped[0] = (int16_t)(wheel_velocity_cmd[LEFT]);
    goal_rpm_spped[1] = (int16_t)(wheel_velocity_cmd[RIGHT]);

    return goal_rpm_spped;
}

void MakeMDRobotMessage1(PID_PNT_MAIN_DATA_t *pData)
{
    static bool first_cal = false;
    static ros::Time previous_time;
    double interval_time;

    ros::Time curr_time = ros::Time::now();

    interval_time = curr_time.toSec() - previous_time.toSec();
    previous_time = curr_time;

    md_robot_msg_pid_pnt_main_data.interval_time = interval_time;
    md_robot_msg_pid_pnt_main_data.motor1_pos = pData->mtr_pos_id1;
    md_robot_msg_pid_pnt_main_data.motor2_pos = pData->mtr_pos_id2;
    md_robot_msg_pid_pnt_main_data.motor1_rpm = pData->rpm_id1;
    md_robot_msg_pid_pnt_main_data.motor2_rpm = pData->rpm_id2;

    md_robot_msg_pid_pnt_main_data.motor1_state = pData->mtr_state_id1.val;
    md_robot_msg_pid_pnt_main_data.motor2_state = pData->mtr_state_id2.val;

    md_robot_msg_pid_pnt_main_data.input_voltage = (float)(curr_pid_io_monitor.input_voltage / 10.0);

#ifdef ENABLE_MD_MESSAGE
    ROS_INFO("interval time1: %f, pos1: %d, pos2: %d, rpm1: %d rpm2: %d, input voltage: %f\r\n",\
                interval_time, md_robot_msg_pid_pnt_main_data.motor1_pos, md_robot_msg_pid_pnt_main_data.motor2_pos, md_robot_msg_pid_pnt_main_data.motor1_rpm, md_robot_msg_pid_pnt_main_data.motor2_rpm, md_robot_msg_pid_pnt_main_data.input_voltage);
#endif
}

// MDUI
void MakeMDRobotMessage2(PID_ROBOT_MONITOR_t *pData)
{
    static bool first_cal = false;
    static ros::Time previous_time;
    double interval_time;

    ros::Time curr_time = ros::Time::now();

    interval_time = curr_time.toSec() - previous_time.toSec();
    previous_time = curr_time;

    md_robot_msg_pid_robot_monitor.interval_time = interval_time;
    md_robot_msg_pid_robot_monitor.x_pos = pData->lTempPosi_x;
    md_robot_msg_pid_robot_monitor.y_pos = pData->lTempPosi_y;
    md_robot_msg_pid_robot_monitor.angule = pData->sTempTheta;

    if(robotParamData.reverse_direction == 0) {
        md_robot_msg_pid_robot_monitor.US_1 = pData->byUS1;
        md_robot_msg_pid_robot_monitor.US_2 = pData->byUS2;
    }
    else {
        md_robot_msg_pid_robot_monitor.US_1 = pData->byUS2;
        md_robot_msg_pid_robot_monitor.US_2 = pData->byUS1;
    }

    md_robot_msg_pid_robot_monitor.platform_state = pData->byPlatStatus.val;
    md_robot_msg_pid_robot_monitor.linear_velocity = pData->linear_velocity;
    md_robot_msg_pid_robot_monitor.angular_velocity = pData->angular_velocity;
    md_robot_msg_pid_robot_monitor.input_voltage = (float)(curr_pid_robot_monitor2.sVoltIn / 10.0);

#ifdef ENABLE_MD_MESSAGE
    ROS_INFO("interval time2: %f, input_voltage: %f\r\n", interval_time, md_robot_msg_pid_robot_monitor.input_voltage);
#endif    
}

/////////// the end of file
