#include <ros/ros.h>
#include "math.h"
#include "sensorcallback.h"
#include "formation_control.h"
#include "passobstacles.h"
#include <iostream>
#include <sstream>
#include <cmath>

using namespace std;

//start positions
float central_x=-2;
float central_y=3;
float central_z=1.8;
float central_w=0;
float gap=2.5;

int main(int argc, char** argv)
{
    obstacle_avoidance four_uav;
    four_uav.mode_init=2;
    four_uav.mode_transit=0;
    four_uav.ja.x=-100;
    four_uav.jb.x=-100;
    four_uav.process=0;

    ros::init(argc, argv, "dynamic_formation");
    ros::NodeHandle na;

    geometry_msgs::Polygon destination;
    geometry_msgs::Pose test_msgs;
    geometry_msgs::Pose obst;

    ros::Subscriber sub= na.subscribe<sensor_msgs::Joy>("/joy",1,joyCallback);
    ros::Subscriber scan_sub1=na.subscribe<sensor_msgs::LaserScan>("/uav/uav1/scan",1,scan1Callback);
    ros::Subscriber scan_sub2=na.subscribe<sensor_msgs::LaserScan>("/uav/uav2/scan",1,scan2Callback);
    ros::Subscriber scan_sub3=na.subscribe<sensor_msgs::LaserScan>("/uav/uav3/scan",1,scan3Callback);
    ros::Subscriber scan_sub4=na.subscribe<sensor_msgs::LaserScan>("/uav/uav4/scan",1,scan4Callback);
    ros::Subscriber loca_sub1=na.subscribe<geometry_msgs::PoseStamped>("/uav/uav1/ground_truth_to_tf/pose",1,poseCallback1);
    ros::Subscriber loca_sub4=na.subscribe<geometry_msgs::PoseStamped>("/uav/uav4/ground_truth_to_tf/pose",1,poseCallback4);

    ros::Publisher pub= na.advertise<geometry_msgs::Polygon>("uav_positions",100);
    ros::Publisher test=na.advertise<geometry_msgs::Pose>("test",100);
    ros::Publisher m2=na.advertise<geometry_msgs::Pose>("m2position",100);

    ros::Rate loop_rate(2);


    while (ros::ok())
    {
        //joystick control states
        central_x +=-array[3];   //each gamepad axes set to different location axis
        central_y +=array[4];
        central_z +=array[1];
        central_w +=array[0]*angle_max;
        four_uav.mode_init+=button[1];
        formation_mode +=button[2];    //odd mode value is line formation, even mode value is square formation
        gap +=0.5*array[7];

        float alpha= central_w/180; //line o3.92984724045rientation angle in radius
        float beta= central_w/180 + 3.1415926/4;   //square orientation angle in radius

        if (formation_mode % 2 != 0)    //even mode value, form a square
        {
            form_square(central_x,central_y,central_z,gap,beta);
        }
        else    // odd mode value, form a line
        {
            form_line( central_x, central_y, central_z, gap, alpha);
        }

        four_uav.detect_obstacle();

        four_uav.M1.x=location1.x+four_uav.M1.range*cos(-four_uav.M1.radius);
        four_uav.M1.y=location1.y+four_uav.M1.range*sin(-four_uav.M1.radius);
        four_uav.M2.x=location4.x+four_uav.M2.range*cos(-four_uav.M2.radius);
        four_uav.M2.y=location4.y+four_uav.M2.range*sin(-four_uav.M2.radius);

        if (four_uav.mode_init %2 !=0)
        {
            four_uav.obstacle_angle=atan((four_uav.M2.y-four_uav.M1.y)/(four_uav.M2.x-four_uav.M1.x));
            four_uav.P1.x=0.5*(four_uav.M1.x+four_uav.M2.x)+5*sin(four_uav.obstacle_angle);
            four_uav.P1.y=0.5*(four_uav.M1.y+four_uav.M2.y)-5*cos(four_uav.obstacle_angle);
            four_uav.P2.x=0.5*(four_uav.M1.x+four_uav.M2.x)-5*sin(four_uav.obstacle_angle);
            four_uav.P2.y=0.5*(four_uav.M1.y+four_uav.M2.y)+5*cos(four_uav.obstacle_angle);
            alpha=four_uav.obstacle_angle;
            beta= central_w/180 + 3.1415926/4;
        }

        else if (four_uav.mode_init ==0)
        {
            alpha=alpha;
        }

        //obstacle avoidance
        else
        {
            if (four_uav.process==0)
            {
                four_uav.pass_obstacle1(four_uav.P1.x,four_uav.P1.y,gap,alpha); //UAV1 UAV2 besides P1
            }
            if (abs(location1.x-four_uav.ja.x)<0.4 && four_uav.process==1)
            {
                four_uav.pass_obstacle2(four_uav.P1.x,four_uav.P1.y,four_uav.P2.x,four_uav.P2.y,gap,alpha); //UAV1 UAV2 besides P2
            }
            else if(abs(location1.y-four_uav.jb.y)<0.4 && four_uav.process==2)
            {
                four_uav.pass_obstacle3(four_uav.P1.x,four_uav.P1.y,four_uav.P2.x,four_uav.P2.y,gap,beta);  //UAV1 UAV2 left float of P2
            }
            else if(abs(location1.x-four_uav.jc.x)<0.4 && four_uav.process==3)
            {
                four_uav.pass_obstacle4(four_uav.P1.x,four_uav.P1.y,four_uav.P2.x,four_uav.P2.y,gap,alpha); //UAV3 UAV4 besides P1
            }
            else if(abs(location4.x-four_uav.jd.x)<0.4 && four_uav.process==4)
            {
                four_uav.pass_obstacle5(four_uav.P2.x,four_uav.P2.y,gap,alpha); //UAV3 UAV4 besides P2
            }
            else if (abs(location4.y-four_uav.je.y)<0.4 && four_uav.process==5)
            {
                four_uav.pass_obstacle6(four_uav.P2.x,four_uav.P2.y,gap,beta);  //UAV3 UAV4 right float of P2
                central_x=four_uav.P2.x;
                central_y=four_uav.P2.y;
            }
            uav1.x=four_uav.fly1.x;uav1.y=four_uav.fly1.y;
            uav2.x=four_uav.fly2.x;uav2.y=four_uav.fly2.y;
            uav3.x=four_uav.fly3.x;uav3.y=four_uav.fly3.y;
            uav4.x=four_uav.fly4.x;uav4.y=four_uav.fly4.y;
        }

        destination.points.resize(4);   //its dynmaic array. Array needs to be resize as its 0 size at the beginning
        destination.points[0].x=uav1.x;
        destination.points[0].y=uav1.y;
        destination.points[0].z=uav1.z;

        destination.points[1].x=uav2.x;
        destination.points[1].y=uav2.y;
        destination.points[1].z=uav2.z;

        destination.points[2].x=uav3.x;
        destination.points[2].y=uav3.y;
        destination.points[2].z=uav3.z;

        destination.points[3].x=uav4.x;
        destination.points[3].y=uav4.y;
        destination.points[3].z=uav4.z;

        test_msgs.position.x=four_uav.M1.x;
        test_msgs.position.y=four_uav.M1.y;
        test_msgs.orientation.x= four_uav.M2.x;
        test_msgs.orientation.y= four_uav.M2.y;

        obst.position.x=four_uav.M2.x;
        obst.position.y=four_uav.M2.y;
        obst.position.z=four_uav.M2.radius;

        test.publish(test_msgs);
        m2.publish(obst);

        pub.publish(destination);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



//time delay
//There are always a delay between subscribe joy data and publish message
//select correct message type
