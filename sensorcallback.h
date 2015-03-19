#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>
#include <iostream>

//joystick variables
double yaw_speed;
double array[8];
int button[8];
double maximum=1;
double angle_max=10;

//laser scan
double range[1080],range1[1080],range2[1080],range3[1080],range4[1080];


struct movement
{
    double x,y,z,yaw;
}destination, location, location1,location2,location3,location4,velocity,error,derror,state_error;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
    location.x=pos->pose.position.x;
    location.y=pos->pose.position.y;
    location.z=pos->pose.position.z;
}

void poseCallback1(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
    location1.x=pos->pose.position.x;
    location1.y=pos->pose.position.y;
}

void poseCallback2(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
    location2.x=pos->pose.position.x;
    location2.y=pos->pose.position.y;
}

void poseCallback3(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
    location3.x=pos->pose.position.x;
    location3.y=pos->pose.position.y;
}


void poseCallback4(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
    location4.x=pos->pose.position.x;
    location4.y=pos->pose.position.y;
}

/*void destCallback(const geometry_msgs::Polygon::ConstPtr& dest,int n)    //UAV1 destination
{
    int uav_num=n;
    destination.x=dest->points[n].x;
    destination.y=dest->points[n].y;
    destination.z=dest->points[n].z;
}
*/

void destCallback1(const geometry_msgs::Polygon::ConstPtr& dest)    //UAV1 destination
{
    destination.x=dest->points[0].x;
    destination.y=dest->points[0].y;
    destination.z=dest->points[0].z;
}

void destCallback2(const geometry_msgs::Polygon::ConstPtr& dest)    //UAV2 destination
{
    destination.x=dest->points[1].x;
    destination.y=dest->points[1].y;
    destination.z=dest->points[1].z;
}

void destCallback3(const geometry_msgs::Polygon::ConstPtr& dest)    //UAV3 destination
{
    destination.x=dest->points[2].x;
    destination.y=dest->points[2].y;
    destination.z=dest->points[2].z;
}

void destCallback4(const geometry_msgs::Polygon::ConstPtr& dest)    //UAV4 destination
{
    destination.x=dest->points[3].x;
    destination.y=dest->points[3].y;
    destination.z=dest->points[3].z;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    for (int i=0;i<=1079;i++)
    {
        range[i]=laser-> ranges[i];
    }
}

void scan1Callback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    for (int i=0;i<=1079;i++)
    {
        range1[i]=laser-> ranges[i];
    }
}

void scan2Callback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    for (int i=0;i<=1079;i++)
    {
        range2[i]=laser-> ranges[i];
    }
}

void scan3Callback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    for (int i=0;i<=1079;i++)
    {
        range3[i]=laser-> ranges[i];
    }
}

void scan4Callback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    for (int i=0;i<=1079;i++)
    {
        range4[i]=laser-> ranges[i];
    }
}



void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    for (int n=0;n<=7;n++)
    {
        array[n]=joy->axes[n]* maximum;
        button[n]=joy->buttons[n];
    }
}
