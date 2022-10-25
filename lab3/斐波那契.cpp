#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

#define PI 3.14159265358979323846

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_star");

    ros::NodeHandle n;//handle of the node

    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    srand(time(0));

    ros::Rate rate(1);

    double a =PI/100;

    double b =PI/100;

    double c =PI/100;

    int iterator = 0;
    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = b;
        msg.angular.z = PI/2;
        iterator=iterator+1;
        if(iterator){
            iterator=0;
            c=a+b;
            a=b;
            b=c;
        }

        vel_pub.publish(msg);

        rate.sleep();

    }
    return 0;
}