#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#include "sensorcallback.h"
#include "pidcontrol.h"

#include <control_toolbox/pid.h>
#include <sstream>
#include <iostream>

using namespace std;

int n;
double loop_frequency=100;  //loop frequency
double period=1/loop_frequency;    //loop rate
float max_axis=1;
float max_angle=2;
double ratio;

float yaw_angle;
float orientation_z,orientation_w;
int adjust_direction=0;

class laserscan
{
    double angle1,angle2,angle3,angle4,angle5,angle6,angle7,angle8,angle9;
    double Ox,Oy;   //last point before pass obstacle
    double safe_distance;

    public:
    double x1,y1,x2,y2,x3,y3,x4,y4;
    int p;  //decide which two points are detecting
    int auto_avoid;     //decide autuo avoid mode
    int obst_count;
    double phi; //angle of the obstacle
    double vx,vy;   //set adjusting speed vector to UAV
    double adjust_velocity;
    double D1,D2;   //selected point distances

    struct obstacle
    {
        double distance, angle,x,y;
    }point1,point2,point3,point4,point5;

    double find_obstacle_direction(double d2,double d3,double d4,double x,double y,int a);
    double find_obstacle_direction2(double, double, double,double, int);
    int detect_obstacle(double d1,double d2,double d3,double d4,double d5,double d6,double d7,double d8,double d9);
};

int laserscan::detect_obstacle(double d1,double d2,double d3,double d4,double d5,double d6,double d7,double d8,double d9)
{
    safe_distance=2;
    p=0;
    if (d1<safe_distance && d2<safe_distance) {p=1;D1=d1;D2=d2;}
    else if (d2<safe_distance && d3<safe_distance) {p=2;D1=d2;D2=d3;}
    else if (d3<safe_distance && d4<safe_distance) {p=3;D1=d3;D2=d4;}
    else if (d4<safe_distance && d5<safe_distance) {p=4;D1=d4;D2=d5;}
    else if (d5<safe_distance && d6<safe_distance) {p=5;D1=d5;D2=d6;}
    else if (d6<safe_distance && d7<safe_distance) {p=6;D1=d6;D2=d7;}
    else if (d7<safe_distance && d8<safe_distance) {p=7;D1=d7;D2=d8;}
    else if (d8<safe_distance && d9<safe_distance) {p=8;D1=d8;D2=d9;}
    else p=0;
    return D1,D2,p;
}

double laserscan::find_obstacle_direction(double d2,double d3,double d4,double x,double y,int a)
{
    adjust_velocity=3;
    angle2= -3.1415926/6;
    angle3= 0;
    angle4= 3.1415926/6;

    //find out obstacle angle phi
    x3=x+d3;
    y3=y;
    x2=x+0.866*d2;
    y2=y+0.5*d2;
    x4=x+0.866*d4;
    y4=y-0.5*d4;

    if (d2<d4)
    {
        phi=atan((y3-y2)/(x3-x2));
    }
    else
    {
        phi=atan((y4-y3)/(x4-x3));
    }

    vx=adjust_velocity*cos(phi);
    vy=adjust_velocity*sin(phi);
    // inverse adjusting velocity direction
    if (a%2 !=0)    {vx=-vx;vy=-vy;}
    else {vx=vx;vy=vy;}

    return vx,vy;
}

double laserscan::find_obstacle_direction2(double D1,double D2,double x,double y,int c)
{
    adjust_velocity=3;
    p=c;
    angle1=-3.1415926*2/3;
    angle2=-3.1415926/4;
    angle3=-3.1415926/3;
    angle4=-3.1415926/6;
    angle5= 0;
    angle6= 3.1415926/6;
    angle7= 3.1415926/3;
    angle8= 3.1415926/4;
    angle9= 3.1415926*2/3;

    switch(p)
    {
        case 1:
        x1=x-0.5*D1;y1=y+0.866*D1;
        x2=x;y2=y+D2;
        break;
        case 2:
        x1=x ;y1=y+D1;
        x2=x+0.5*D2;y2=y+0.866*D2;
        break;
        case 3:
        x1=x+0.5*D1 ;y1=y+0.866*D1;
        x2=x+0.866*D2;y2=y+0.5*D2;
        break;
        case 4:
        x1=x+0.866*D1 ;y1=y+0.5*D1;
        x2=x+D2;y2=y;
        break;
        case 5:
        x1=x+D1 ;y1=y;
        x2=x+0.866*D2;y2=y-0.5*D2;
        break;
        case 6:
        x1=x+0.886*D1 ;y1=y-0.5*D1;
        x2=x+0.5*D2;y2=y-0.886*D2;
        break;
        case 7:
        x1=x+0.5*D1 ;y1=y+0.886*D1;
        x2=x;y2=y-0.5*D2;
        break;
        case 8:
        x1=x ;y1=y-D1;
        x2=x-0.5*D2;y2=y-0.886*D2;
        break;
    }

    phi= atan((y2-y1)/(x2-x1));
    vx=adjust_velocity*cos(phi);
    vy=adjust_velocity*sin(phi);

    return vx,vy;
}


int main(int argc, char** argv)
{
    pidcontrol longitude, latitude, altitude, yaw_angle;
    longitude.setparam(1.2,0,0.6,3,6);
    latitude.setparam(1.2,0,0.6,3,6);
    altitude.setparam(0.2,0,0.1,2,3);
    yaw_angle.setparam(1,0,0,5,5);

    destination.z=2;

    geometry_msgs::Twist msg;
    geometry_msgs::Point desiredp;
    geometry_msgs::Point e_msg;
    geometry_msgs::Pose test_msg;

    laserscan single_detect;
    single_detect.obst_count=0;
    single_detect.adjust_velocity=3;

    ros::init(argc, argv, "single_flight");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/uav/cmd_vel",1000);
    ros::Publisher point= nh.advertise<geometry_msgs::Point>("/uav_point",1000);
    ros::Publisher e=nh.advertise<geometry_msgs::Point>("/error_signal",1000);
    ros::Publisher test=nh.advertise<geometry_msgs::Pose>("/test",1000);
    ros::Subscriber sub_loca= nh.subscribe<geometry_msgs::PoseStamped>("/uav/ground_truth_to_tf/pose",1,poseCallback);
    ros::Subscriber sub_destination= nh.subscribe<sensor_msgs::Joy>("/joy",1,joyCallback);
    ros::Subscriber sub_scan=nh.subscribe<sensor_msgs::LaserScan>("/uav/scan",1,scanCallback);

    ros::Rate loop_rate(loop_frequency);

    ros::Time last_time=ros::Time::now();

    while (ros::ok())
    {
        ros::Time time=ros::Time::now();

        destination.x +=-0.04*array[3];   //each gamepad axes set to different location axis
        destination.y +=0.04*array[4];
        destination.z +=0.1*array[1];
        adjust_direction +=button[0];

        //differences
        error.x=destination.x-location.x;
        error.y=destination.y-location.y;
        error.z=destination.z-location.z;

        longitude.updatePid3(error.x,state_error.x,ros::Duration(period));
        latitude.updatePid3(error.y,state_error.y,ros::Duration(period));
        altitude.updatePid3(error.z,state_error.z,ros::Duration(period));

        velocity.x=longitude.cmd_;
        velocity.y=latitude.cmd_;
        velocity.z=altitude.cmd_;
        single_detect.detect_obstacle(range[59],range[179],range[299],range[419],range[539],range[659],range[779],range[899],range[1019]);

        if (single_detect.p !=0)
        {
            single_detect.find_obstacle_direction2(single_detect.D1,single_detect.D2,location.x,location.y,single_detect.p);
            velocity.x=single_detect.vx;
            velocity.y=single_detect.vy;
            //let UAV hover after obstacle
            destination.x=location.x+2;//*velocity.x/sqrt(pow(velocity.x,2)+pow(velocity.y,2));
            destination.y=location.y+2;//*velocity.y/sqrt(pow(velocity.x,2)+pow(velocity.y,2));
        }
        //publish velocity message
        msg.linear.x=velocity.x;
        msg.linear.y=velocity.y;
        msg.linear.z=velocity.z;
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;

        desiredp.x=destination.x;
        desiredp.y=destination.y;
        desiredp.z=destination.z;

        e_msg.x=error.x;
        e_msg.y=error.y;
        e_msg.z=error.z;

        test_msg.position.x=single_detect.x1;
        test_msg.position.y=single_detect.y1;
        test_msg.orientation.x=single_detect.x2;
        test_msg.orientation.y=single_detect.y2;

        state_error.x=error.x;
        state_error.y=error.y;
        state_error.z=error.z;
        //state_error.yaw=error.yaw;
        single_detect.obst_count=0;
        ros::Time last_time=ros::Time::now();

        pub.publish(msg);   //velocity message
        point.publish(desiredp);    //destination info
        e.publish(e_msg);   //error infor
        test.publish(test_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
