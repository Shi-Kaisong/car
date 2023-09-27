#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#define Type 1  //1:阿克慢，4：四轮四驱

ros::Subscriber cmd_vel_sub;
ros::Publisher cmd_vel_pub;

using namespace std;

double vx = 0.0;
double vth = 0.0;

//filling the velocity
geometry_msgs::Twist velocity;

//阿克慢前后轮距离
#define WHELLBASE 0.8

//计算阿克慢转角弧度
float angz_to_angle(float Vx,float Vz)
{
    float R=0;
    if(Vx==0 || Vz==0)
    {
    return 0;
    }
    
    R = Vx/Vz;
    return atan(WHELLBASE/R);
}

void cmd_velCallback(const geometry_msgs::TwistStamped &msg)
{
    vx = msg.twist.linear.x;
    vth = msg.twist.angular.z;

    velocity.linear.x = vx;
#if   Type == 1
    velocity.angular.z = angz_to_angle(vx,vth);
#elif Type == 4
    velocity.angular.z = vth;
#endif

    cout << "vx: " << velocity.linear.x << endl;
    cout << "vth: " << velocity.angular.z << endl;

    cmd_vel_pub.publish(velocity);
    ros::spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_trans");
    ros::NodeHandle n;

    cmd_vel_sub = n.subscribe("/twist_cmd", 10, cmd_velCallback);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);//发布速度

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}