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
float central_x=-7;
float central_y=0;
float central_z=2;
float central_w=0;
float state_x,state_y;  //store desired position behind obstacles
float gap=2.5;
int shabi=0;

int main(int argc, char** argv)
{
    obstacle_avoidance bypass;
    bypass.mode_init=0;
    bypass.mode_transit=0;
    bypass.process=0;

    formation_mode=0;   //decide which formation

    ros::init(argc, argv, "avoid_obstacles");
    ros::NodeHandle nav;

    geometry_msgs::Polygon destination;
    geometry_msgs::Pose test_msgs;

    ros::Subscriber sub= nav.subscribe<sensor_msgs::Joy>("/joy",1,joyCallback);
    ros::Subscriber scan_sub1=nav.subscribe<sensor_msgs::LaserScan>("/uav/uav1/scan",1,scan1Callback);
    ros::Subscriber scan_sub2=nav.subscribe<sensor_msgs::LaserScan>("/uav/uav2/scan",1,scan2Callback);
    ros::Subscriber scan_sub3=nav.subscribe<sensor_msgs::LaserScan>("/uav/uav3/scan",1,scan3Callback);
    ros::Subscriber scan_sub4=nav.subscribe<sensor_msgs::LaserScan>("/uav/uav4/scan",1,scan4Callback);
    ros::Subscriber loca_sub1=nav.subscribe<geometry_msgs::PoseStamped>("/uav/uav1/ground_truth_to_tf/pose",1,poseCallback1);
    ros::Subscriber loca_sub2=nav.subscribe<geometry_msgs::PoseStamped>("/uav/uav1/ground_truth_to_tf/pose",1,poseCallback2);
    ros::Subscriber loca_sub3=nav.subscribe<geometry_msgs::PoseStamped>("/uav/uav1/ground_truth_to_tf/pose",1,poseCallback3);
    ros::Subscriber loca_sub4=nav.subscribe<geometry_msgs::PoseStamped>("/uav/uav4/ground_truth_to_tf/pose",1,poseCallback4);

    ros::Publisher pub= nav.advertise<geometry_msgs::Polygon>("uav_positions",100);
    ros::Publisher test=nav.advertise<geometry_msgs::Pose>("test",100);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        //joystick control states
        central_x +=-0.01*array[3];   //each gamepad axes set to different location axis
        central_y +=0.01*array[4];
        central_z +=array[1];
        central_w +=array[0]*angle_max;
        bypass.mode_init+=array[2];
        formation_mode +=button[2];    //odd mode value is line formation, even mode value is square formation
        gap +=0.5*array[7];

        double alpha= central_w*3.1415926/180; //line o3.92984724045rientation angle in radius
        double beta= central_w/180 + 3.1415926/4;   //square orientation angle in radius

        bypass.full_detect(alpha);

       if (bypass.mode_transit %2 ==1 && bypass.mode_init !=0)   //transit mode initiate
        {
            central_x =central_x+0.01;
            central_y =central_y;
            shabi+=1;
            //bypass.mode_transit +=1;
            if (bypass.L1.range>12 && bypass.L2.range>12)
            {
                bypass.mode_transit+=1;
                //central_x+=0;
                //central_y+=15;
            }
        }

        //reset to manual mode, when wrong laser scan occurred
        else if (bypass.L1.range>5 && bypass.L2.range>5 && bypass.mode_transit%2 ==1)
        {
            bypass.mode_transit+=1;
        }
		
        if (formation_mode % 2 == 0)    //even mode value, form a square
        {
            form_square(central_x,central_y,central_z,gap,beta);
        }
        else    // odd mode value, form a line
        {
            form_line( central_x, central_y, central_z, gap, alpha);
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

        test_msgs.position.x=bypass.L1.x;
        test_msgs.position.y=bypass.L1.y;
        test_msgs.position.z=bypass.mode_init;
        test_msgs.orientation.x=bypass.M1.x;
        test_msgs.orientation.y=bypass.M2.y;
        test_msgs.orientation.z=bypass.M2.x;
        test_msgs.orientation.w=bypass.M2.y;

        test.publish(test_msgs);

        pub.publish(destination);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



//time delay
//There are always a delay between subscribe joy data and publish message

//select correct message type
