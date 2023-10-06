#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"

#include <iostream>

#include "ros/ros.h"
#include <sstream>
#include <geometry_msgs/Twist.h>

#include <string>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <ctime>
#include <stdint.h>

#include "car_top/car_top.h"//自定义消息类型

#define PI 3.141592653

VCI_BOARD_INFO pInfo;//用来获取设备信息。
using namespace std;

//int speed_step = 0;             //下发的速度，单位:mm/s
//int angle_step = 0;             //下发的角度，也就是can协议里面的数据N，初始角度为0时候数据N为19

short back_wheel_speed = 0.0;   //上报的后轮速度
short turn_angle = 0.0;         //上报的转向角度
short battery_level = 0.0;      //上报的电量百分比
short error_flag = 0.0;         //上报的错误状态
short left_speed = 0.0;         //上报的后左轮速度
short right_speed = 0.0;        //上报的后右轮速度

//send can data to AKM
void SendSpeedToAKM(float lineSpeed,float angle)
{
    int LineSpeed=0,Angle=0;
    LineSpeed = lineSpeed * 1000;        //ros里面的m/s转化为mm/s
    Angle     = angle * 100*180/M_PI;
     //需要发送的帧，结构体设置
    VCI_CAN_OBJ send[1];
    send[0].ID=0x00000001;//根据底盘can协议，发送时候can的ID为0x01
    send[0].SendType=0;
    send[0].RemoteFlag=0;
    send[0].ExternFlag=0;
    send[0].DataLen=8;

    //要写入的数据
    send[0].Data[0] = 0x00000001;
    send[0].Data[1] = 1;
    send[0].Data[2] = LineSpeed      & 0xFF;
    send[0].Data[3] = (LineSpeed>>8) & 0xFF;
    send[0].Data[4] = Angle          & 0xFF;
    send[0].Data[5] = (Angle>>8)     & 0xFF;
    send[0].Data[6] = 0x00;
    send[0].Data[7] = 0x00;
    
    //写入数据
    if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
    {
       //printf("TX data successful!\n");
    }
}

//档位设置函数0：低速档，1：中速档，2：高速档，默认低速档
void SetGear(int gear)
{
     //需要发送的帧，结构体设置
    VCI_CAN_OBJ send[1];
    send[0].ID=0x00000001;//根据底盘can协议，发送时候can的ID为0x01
    send[0].SendType=0;
    send[0].RemoteFlag=0;
    send[0].ExternFlag=0;
    send[0].DataLen=8;

    //要写入的数据
    send[0].Data[0] = 0x01;
    send[0].Data[1] = 0x03;
    send[0].Data[2] = gear;
    send[0].Data[3] = 0x00;
    send[0].Data[4] = 0x00;
    send[0].Data[5] = 0x00;
    send[0].Data[6] = 0x00;
    send[0].Data[7] = 0x00;

    //写入数据
    if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
    {

    }
}

//刹车力度设置函数  0x00-0x05  0x05为急刹 
//默认为 0x00
void SetBrek(int brek) 
{
     //需要发送的帧，结构体设置
    VCI_CAN_OBJ send[1];
    send[0].ID=0x00000001;//根据底盘can协议，发送时候can的ID为0x01
    send[0].SendType=0;
    send[0].RemoteFlag=0;
    send[0].ExternFlag=0;
    send[0].DataLen=8;

    //要写入的数据
    send[0].Data[0] = 0x01;
    send[0].Data[1] = 0x04;
    send[0].Data[2] = brek;
    send[0].Data[3] = 0x00;
    send[0].Data[4] = 0x00;
    send[0].Data[5] = 0x00;
    send[0].Data[6] = 0x00;
    send[0].Data[7] = 0x00;

    //写入数据
    if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
    {

    }
}

/*
mode: 0:常亮 1:闪烁 2:呼吸
light:亮度(0-100)
delay:间隔时间(1-255)对应时间(0.1-25.5s)
R:(0-255)颜色值
G:(0-255)
B:(0-255)
*/
void SetLed(unsigned char mode,unsigned char light,unsigned char delay,unsigned char R,unsigned char G,unsigned char B) 
{
     //需要发送的帧，结构体设置
    VCI_CAN_OBJ send[1];
    send[0].ID=0x00000001;//根据底盘can协议，发送时候can的ID为0x01
    send[0].SendType=0;
    send[0].RemoteFlag=0;
    send[0].ExternFlag=0;
    send[0].DataLen=8;

    //要写入的数据
    send[0].Data[0] = 0x01;
    send[0].Data[1] = 0x05;
    send[0].Data[2] = mode;
    send[0].Data[3] = light;
    send[0].Data[4] = delay;
    send[0].Data[5] = R;
    send[0].Data[6] = G;
    send[0].Data[7] = B;

    //写入数据
    if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
    {

    }
}

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)//速度控制回调
{
    SendSpeedToAKM(twist_aux.linear.x,twist_aux.angular.z);
}

int main(int argc, char **argv)
{
    int i=0,re_size = 0;
    car_top::car_top msg;

    printf(">>this is hello !\r\n");//指示程序已运行
    int num = VCI_OpenDevice(VCI_USBCAN2,0,0);
    if(num == 0 || num == 1)//打开设备
    {
        printf(">>open deivce success!\n");//打开设备成功
    }else
    {
        printf(">>open deivce error %d!\n",num);
        exit(1);
    }
    if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
    {
        printf(">>Get VCI_ReadBoardInfo success!\n");
    }else
    {
        printf(">>Get VCI_ReadBoardInfo error!\n");
        exit(1);
    }

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;//FFFFFFFF全部接收
    config.Filter=2;//接收所有帧  2-只接受标准帧  3-只接受扩展帧
    config.Timing0=0x00;/*波特率500 Kbps  0x00  0x1C*/
    config.Timing1=0x1C;
    config.Mode=0;//正常模式

    if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    }

    if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);

    }
   
    //需要读取的帧，结构体设置
    VCI_CAN_OBJ rev[2500];
    rev[0].SendType=0;
    rev[0].RemoteFlag=0;
    rev[0].ExternFlag=0;
    rev[0].DataLen=8;

    /*
    int m_run0=1;
    pthread_t threadid;
    int ret;
    ret=pthread_create(&threadid,NULL,receive_func,&m_run0);
    int times = 5;
    */

    ros::init(argc, argv, "car_top");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 100, cmd_velCallback);//速度回调
    ros::Publisher car_pub      = n.advertise<car_top::car_top>("/car_message", 10);//自定义消息发布
    ros::Rate loop_rate(50);

    
    //档位设置
    SetGear(0);
    sleep(0.1);
    //刹车力度设置
    SetBrek(0);
    sleep(0.1);
    //灯条设置
    SetLed(1,100,10,255,0,0);
    
    while (ros::ok())
    {
        //读取数据
        re_size = VCI_Receive(VCI_USBCAN2, 0, 0,rev, 2500, 0);
        if(re_size > 0)
        {
            for(int i=0;i< re_size;i++)
            {
                if((rev[i].ID == 0x10)&&(rev[i].Data[0] == 0x02) && (rev[i].Data[1] == 0x02))//can发送02代表接收的数据
                {
                    back_wheel_speed    = (rev[i].Data[3] << 8) | rev[i].Data[2];//上报的后轮速度
                    turn_angle          = (rev[i].Data[5] << 8) | rev[i].Data[4];//上报的转向角度
                    battery_level       = rev[i].Data[6];//上报的电量百分比
                    error_flag          = rev[i].Data[7];//上报的错误状态
                    /*-------------------------------------------------------------------------*/
                    msg.back_wheel_speed = back_wheel_speed;
                    msg.turn_angle = turn_angle/100.00*M_PI/180;
                    msg.battery_level = battery_level;
                    msg.error_flag = error_flag;

                    cout << "后轮速度: " << back_wheel_speed << " mm/s" << endl;
                    cout << "转向弧度: " << turn_angle/100.00*M_PI/180 << endl;
                    cout << "错误状态: " << error_flag << endl; 
                    cout << "电量百分比: " <<battery_level << endl;
                    cout << "------------------------------------------------------"<< endl;

                    car_pub.publish(msg);
                }
                else if((rev[i].ID == 0x11)&&(rev[i].Data[0] == 02) && (rev[i].Data[1] == 02))//can发送02代表接收的数据
                {
                    left_speed    = (rev[i].Data[3] << 8) | rev[i].Data[2];//上报的后轮速度
                    right_speed   = (rev[i].Data[5] << 8) | rev[i].Data[4];//上报的转向角度
                    
                    msg.left_speed  = left_speed;        //上报的后左轮速度
                    msg.right_speed = right_speed;       //上报的后右轮速度

                    cout << "后左轮速度: " << left_speed << endl; 
                    cout << "后右轮速度: " <<right_speed << endl;

                    cout << "------------------------------------------------------"<< endl;
                }
                else if(rev[i].ID == 0x12)//can发送02代表接收的数据
                {
                    msg.front_current   = (short)((rev[i].Data[1] << 8) | rev[i].Data[0]);//上报的后轮速度
                    msg.left_current    = (short)((rev[i].Data[3] << 8) | rev[i].Data[2]);//上报的转向角度
                    msg.right_current   = (short)((rev[i].Data[5] << 8) | rev[i].Data[4]);//上报的转向角度
                    
                    cout << "转向电机电流: " << msg.front_current << endl; 
                    cout << "左后电机电流: " << msg.left_current  << endl;
                    cout << "右后电机电流: " << msg.right_current << endl;

                    cout << "------------------------------------------------------"<< endl;
                }

            }
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
    return 0;
}
