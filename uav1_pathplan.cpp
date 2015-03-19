#include "ros/ros.h"
#include "pidcontrol.h"
#include "sensorcallback.h"
#include <sstream>
#include <iostream>

using namespace std;

double loop_period;

int main(int argc, char** argv)
{
    pidcontrol longitude, latitude, altitude, yaw_angle;    //init pid control for longitude, latitude and altitude

    //set pid control parameter
    longitude.setparam(1.2,0,0.5,3,6);
    latitude.setparam(1.2,0,0.5,3,6);
    altitude.setparam(0.2,0,0.1,2,3);
    geometry_msgs::Twist msg;

    ros::init(argc, argv, "uav1_pathplan");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/uav/uav1/cmd_vel",1000);
    ros::Subscriber sub_loca= nh.subscribe<geometry_msgs::PoseStamped>("/uav/uav1/ground_truth_to_tf/pose",1,poseCallback);
    ros::Subscriber sub_destination= nh.subscribe<geometry_msgs::Polygon>("/uav_positions",1,destCallback1);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        error.x=destination.x-location.x;
        error.y=destination.y-location.y;
        error.z=destination.z-location.z;

        longitude.updatePid3(error.x,state_error.x,ros::Duration(0.01));
        latitude.updatePid3(error.y,state_error.y,ros::Duration(0.01));
        altitude.updatePid3(error.z,state_error.z,ros::Duration(0.01));

        msg.linear.x=longitude.cmd_;
        msg.linear.y=latitude.cmd_;
        msg.linear.z=altitude.cmd_;
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;;

        state_error.x=error.x;
        state_error.y=error.y;
        state_error.z=error.z;

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
