#include "ros/ros.h"
#include <sstream>
#include <iostream>

class pidcontrol
{
    float kp,ki,kd,kyaw;
    float p_term,i_term,d_term;
    float i_limit, cmd_limit;

    public:
    double ddt,dddt;
    double ti,td;   //integration period and differential period
    void setparam(double,double,double,double,double);
    double cmd_;
    double updatePid(double error1, double error2, ros::Duration dt);
    double updatePid2(double , double ,int);
    double updatePid3(double error1, double error2, const ros::Duration& dt);


};

void pidcontrol::setparam(double q,double w,double e,double r,double t)
{
    kp=q;
    ki=w;
    kd=e;
    i_limit=r;  //integral item limit
    cmd_limit=t;    //cmd_ limit
}

double pidcontrol::updatePid(double error1,double error2, ros::Duration dt)
{
    dddt=dt.toSec();
    //error1 is current error, error2 is last error
    p_term= error1;
    //intergration item
    i_term += 0.5*(error1+error2)*dt.toSec();
    //differential item
    d_term= (error1-error2)/dt.toSec();

    if (i_limit>0)      //set limit to integral item
    {
        if (i_term> i_limit) i_term=i_limit;
        if (i_term< -i_limit) i_term=-i_limit;
    }

    cmd_=kp*p_term+ki*i_term+kd*d_term;

    if (cmd_limit>0)
    {
        if (cmd_ >cmd_limit) cmd_=cmd_limit;
        if (cmd_ <-cmd_limit) cmd_=-cmd_limit;
    }
    return cmd_,dddt;
}

double pidcontrol::updatePid2(double error1,double error2, int rate)
{
    //error1 is current error, error2 is last error
    p_term= error1;
    //intergration item
    i_term += 0.5*(error1+error2)/rate;
    //differential item
    d_term= (error1-error2)*rate;

    if (i_limit>0)      //set limit to integral item
    {
        if (i_term> i_limit) i_term=i_limit;
        if (i_term< -i_limit) i_term=-i_limit;
    }

    cmd_=kp*p_term+ki*i_term+kd*d_term;

    if (cmd_limit>0)
    {
        if (cmd_ >cmd_limit) cmd_=cmd_limit;
        if (cmd_ <-cmd_limit) cmd_=-cmd_limit;
    }
    return cmd_,dddt;

}

double pidcontrol::updatePid3(double error1,double error2, const ros::Duration& dt)
{
    ddt=dt.toSec();
    ti=dt.toSec();  //integration period
    td=10*dt.toSec();   //diffrential period
    //error1 is current error, error2 is previous error
    p_term= error1;
    //intergration item
    i_term += 0.5*(error1+error2)*ti;
    //differential item
    d_term= (error1-error2)/td;

    if (i_limit>0)      //set limit to integral item
    {
        if (i_term> i_limit) i_term=i_limit;
        if (i_term< -i_limit) i_term=-i_limit;
    }

    cmd_=kp*p_term+ki*i_term+kd*d_term;

    if (cmd_limit>0)
    {
        if (cmd_ >cmd_limit) cmd_=cmd_limit;
        if (cmd_ <-cmd_limit) cmd_=-cmd_limit;
    }
    return cmd_,ddt;
}
