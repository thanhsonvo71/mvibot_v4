#include"../../common/libary/libary_basic.h"
#include"../../common/libary/libary_ros.h"
using namespace std;
//
extern string mvibot_seri;
extern nav_msgs::Odometry odom_wheel;
extern sensor_msgs::Imu imu_msg;
extern float x_wheel,y_wheel,theta_wheel;
extern float vx_wheel,vy_wheel,vth_wheel;
extern float distance_robot;
extern float distance_wheel_right;
extern float distance_wheel_left;
extern float R,L,vr,vl;
extern  int mvibot_sensor_ready;
//
void pub_imu(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<sensor_msgs::Imu>("/"+mvibot_seri+"/imu",1);
    static float creat_fun=0;
        if(creat_fun==1)
        {
                pub.publish(imu_msg);
        } else creat_fun=1;  
}
void pub_odom_wheel(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<nav_msgs::Odometry>("/"+mvibot_seri+"/odom_wheel",1);
    static float creat_fun=0;
        if(creat_fun==1)
        {
                pub.publish(odom_wheel);
        } else creat_fun=1;
}
void caculate_and_send_odom(){
        // caculate
        static float ts_local=0.05;
        static  geometry_msgs::Quaternion odom_wheel_quat,odom_wheel_laser_quat;
        static  geometry_msgs::Quaternion quat_msg;
        if(mvibot_sensor_ready==0){
            vr=0;
            vl=0;
            imu_msg.angular_velocity.x=0.0;
            imu_msg.angular_velocity.y=0.0;
            imu_msg.angular_velocity.z=0.0;
        }
        // time set
        odom_wheel.header.stamp = ros::Time::now();
        imu_msg.header.stamp=ros::Time::now();
        // odom wheel
        x_wheel=x_wheel+R/2*(vl+vr)*cos(theta_wheel)*ts_local;
        y_wheel=y_wheel+R/2*(vl+vr)*sin(theta_wheel)*ts_local;
        theta_wheel=theta_wheel+R/L*(vr-vl)*ts_local;
        vy_wheel=0;
        vx_wheel=R/2*(vr+vl);
        vth_wheel=R/L*(vr-vl);
        odom_wheel.pose.pose.position.x=x_wheel;
        odom_wheel.pose.pose.position.y=y_wheel;
        odom_wheel_quat=tf::createQuaternionMsgFromYaw(theta_wheel);
        odom_wheel.pose.pose.orientation=odom_wheel_quat;
        odom_wheel.twist.twist.linear.x = vx_wheel;
        odom_wheel.twist.twist.linear.y = vy_wheel;
        odom_wheel.twist.twist.angular.z = vth_wheel;
        // distance robot have been move
        distance_robot+=fabs(vx_wheel)*(float)ts_local;
        distance_wheel_left+=fabs(vl*R)*(float)ts_local;
        distance_wheel_right+=fabs(vr*R)*(float)ts_local;
        //send topic
        pub_odom_wheel();
        pub_imu();
}