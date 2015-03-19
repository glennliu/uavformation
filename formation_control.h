#include <ros/ros.h>
#include "math.h"
#include <iostream>
#include <sstream>
#include <cmath>

int formation_mode;

struct position
{
    float x,y,z;
}uav1,uav2,uav3,uav4;


void form_square(double xc,double yc,double zc,double g,double b)
{
        uav1.x=xc-1.41*g*cos(b);
        uav1.y=yc+1.41*g*sin(b);
        uav1.z=zc;

        uav2.x=xc-1.41*g*sin(b);
        uav2.y=yc-1.41*g*cos(b);
        uav2.z=zc;

        uav3.x=xc+1.41*g*cos(b);
        uav3.y=yc-1.41*g*sin(b);
        uav3.z=zc;

        uav4.x=xc+1.41*g*sin(b);
        uav4.y=yc+1.41*g*cos(b);
        uav4.z=zc;
}

void form_line(double xc,double yc,double zc,double g,double a)
{
        uav1.x=xc-3*g*cos(a);
        uav1.y=yc-3*g*sin(a);
        uav1.z=zc;

        uav2.x=xc-g*cos(a);
        uav2.y=yc-g*sin(a);
        uav2.z=zc;

        uav3.x=xc+g*cos(a);
        uav3.y=yc+g*sin(a);
        uav3.z=zc;

        uav4.x=xc+3*g*cos(a);
        uav4.y=yc+3*g*sin(a);
        uav4.z=zc;
}
