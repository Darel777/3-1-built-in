#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

#define PI 3.14159265358979323846

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_star");

    ros::NodeHandle n;

    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    srand(time(0));

    ros::Rate rate(2);

    int iterator = 0;
    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 3;
        iterator++;

        if(iterator == 2){
            iterator = 0;
            msg.linear.x = 0;
            msg.angular.z = PI*(8.0/5);
        }

        vel_pub.publish(msg);

        rate.sleep();

    }
    return 0;
}
