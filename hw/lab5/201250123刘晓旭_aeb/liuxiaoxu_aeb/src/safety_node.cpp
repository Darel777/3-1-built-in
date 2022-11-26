#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class Safety {
private:
    ros::NodeHandle n;
    double speed;
    ros::Subscriber LidarSub; 
    ros::Subscriber OdomSub; 
    ros::Publisher AckermannPub; 
    ros::Publisher BreakPub; 
    ackermann_msgs::AckermannDriveStamped ackermann;
    std_msgs::Bool Break;
    float speed_x;  
    float min_ttc; 
    void ExecuteAeb()
    {
        ackermann.drive.speed = 0; 
        Break.data = true; 
        AckermannPub.publish(ackermann); 
        BreakPub.publish(Break);
        ROS_INFO("AEB ACtivated");
    }
    void GetSpeedCallback(nav_msgs::OdometryConstPtr odom_msg_ptr)
    {
        speed_x = odom_msg_ptr->twist.twist.linear.x; 
    }
    void Ttcalculation(sensor_msgs::LaserScanConstPtr scan_msg_ptr)
    {
        float distance;
        float velocity; 
        float current_angle = scan_msg_ptr->angle_min; 
        ROS_INFO("Angle_MIN: %f", scan_msg_ptr->angle_min);
        float ttc; 
        

        for(int i=0; i < scan_msg_ptr->ranges.size(); i++){
            distance = scan_msg_ptr->ranges[i];
            
            velocity = speed_x * cos(current_angle);
            ttc = abs(distance/(velocity+0.00001)); 
            
            current_angle += scan_msg_ptr->angle_increment;
            if(ttc < min_ttc)
            {
                ExecuteAeb(); 
                break;
            }
            
        }
    }
public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        min_ttc = 0.3; 
        LidarSub = n.subscribe("scan", 10, &Safety::Ttcalculation, this);
        OdomSub = n.subscribe("odom", 10, &Safety::GetSpeedCallback, this);
        AckermannPub = n.advertise<ackermann_msgs::AckermannDriveStamped>("brake",10);
        BreakPub = n.advertise<std_msgs::Bool>("brake_bool",10);
        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        speed = 0.0;
    }
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {

    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}