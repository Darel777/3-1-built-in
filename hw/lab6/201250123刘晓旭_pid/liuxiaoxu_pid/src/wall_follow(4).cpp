#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class Controller{
private:
    ros::NodeHandle n;
    
    ros::Subscriber LidarSub; 

    ros::Publisher AckermannPub; 
    ackermann_msgs::AckermannDriveStamped ackermann;
    double K_P = 5; 
    double K_I = 0;
    double K_D = 0.2;
    double k_L ;
    double steer_angle;
    double last_time_reading = ros::Time::now().toSec();
    double current_time_reading = ros::Time::now().toSec(); 
    double look_ahead_distance = 0.05; 
    double error_dist = 0; 
    double last_error_dist = 0;
    double integral = 0; 
    double speed;
    double initial_time = ros::Time::now().toSec(); 
    

    void ErrorCalculation(sensor_msgs::LaserScanConstPtr scan_msg_ptr)
    {
        double a_dist;
        double b_dist;
        double c_dist; 
        double a1_angle;
        double a2_angle;
        double D_t;
        double D_ta;
        double D_tc;
        double D_t1; 
        double angle_increment = 0.005825;// create car crush
        double theta_a = angle_increment * 195;
        double theta_c = angle_increment * 165; 
        b_dist = scan_msg_ptr->ranges[809];
        a_dist = scan_msg_ptr->ranges[614]; 
        c_dist = scan_msg_ptr->ranges[644]; 
        a1_angle = atan((a_dist*cos(theta_a)-b_dist)/(a_dist*sin(theta_a)));
        D_ta = b_dist*cos(a1_angle); 


        a2_angle = atan((c_dist*cos(theta_c)-b_dist)/(c_dist*sin(theta_c)));
        D_tc = b_dist*cos(a2_angle); 

        D_t = 0.5*(D_ta + D_tc);
  
        error_dist = 1.35 -D_t;
        if(ros::Time::now().toSec() - initial_time < 5){ error_dist *= 0.3;}

        PidController(); 
        }

    void PidController(){ 
        
        float steer_angle; 
        float d_term;
        integral += error_dist; 
        current_time_reading = ros::Time::now().toSec();
        if(current_time_reading - last_time_reading > 0.2){ 
            
        float dt; 
        float d_error;
        dt = current_time_reading - last_time_reading;
        d_error = error_dist - last_error_dist; 
        last_time_reading = current_time_reading;
        last_error_dist = error_dist;
        
        d_term = d_error/dt;
        
        if(current_time_reading - initial_time < 5){ 
            d_term = 0; 
        }
        ROS_INFO("d_error: %f", d_term);
        }
        steer_angle = -K_P * error_dist + K_I * integral + K_D * d_term; 
        if(steer_angle> M_PI){steer_angle = M_PI - 0.015; }
        else if(steer_angle > M_PI-0.005){steer_angle = M_PI - 0.005;}
        if(steer_angle< -M_PI){steer_angle = -M_PI+ 0.015; }
        else if(steer_angle < -M_PI+0.005){steer_angle = -M_PI + 0.005;}
        if(abs(steer_angle) < 0.05){speed = 4.5;} 
        else if(abs(steer_angle) > 0.05 && abs(steer_angle) < 0.10){speed = 3.5;}
        else if(abs(steer_angle) > 0.10 && abs(steer_angle) < 0.15){speed = 3;}
        else if(abs(steer_angle) > 0.15 && abs(steer_angle) < 0.20){speed = 2.5;}
        else if(abs(steer_angle) > 0.20 && abs(steer_angle) < 0.25){speed = 2;}
        else if(abs(steer_angle) > 0.25 && abs(steer_angle) < 0.30){speed = 1.5;}
        else if(abs(steer_angle) > 0.30 && abs(steer_angle) < 0.35){speed = 1;} 
        else if(abs(steer_angle) > 0.35){speed = 0.5;}
        ackermann.drive.speed = speed; 
        ackermann.drive.steering_angle = steer_angle; 
        ackermann.header.frame_id = "laser"; 
        ackermann.header.stamp = ros::Time::now();
        AckermannPub.publish(ackermann); 
    }
public:
    Controller() {
        n = ros::NodeHandle();

        LidarSub = n.subscribe("scan", 1000, &Controller::ErrorCalculation, this);
        AckermannPub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1000);
    }
};
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
    Controller cn;
    Safety sn;
    ros::spin();
    return 0;
}
