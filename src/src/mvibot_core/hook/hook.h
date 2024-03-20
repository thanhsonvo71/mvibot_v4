#include"../../common/libary/libary_basic.h"
#include"../../common/libary/libary_ros.h"
#include"../../common/send_tranfrom/send_tranfrom.h"
// hook
extern float encoder,encoder2;
extern float encoder_offset;
extern float encoder_dir;
extern float d_hook;
extern int hook_switch;
//
using namespace std;
void pub_hook_laser(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<sensor_msgs::LaserScan>("/"+mvibot_seri+"/hook_laser",1);
    static sensor_msgs::LaserScan virtual_laser_hook;
    static float creat_fun=0;
        if(creat_fun==1)
        {
                virtual_laser_hook.header.stamp=ros::Time::now();
                static float dis_x=1.0;
                static float dis_y=0.1;
                for(float i=0;i<dis_x/0.1*2;i++){
                    static float dis_x_2,dis_y_2;
                    dis_x_2=dis_x-i*0.1;
                    dis_y_2=dis_y;
                    //
                    static float stt;
                    stt=(float)(360*(atan2(dis_x_2,dis_y_2)+M_PI)/(2*M_PI));
                    virtual_laser_hook.ranges[stt]=sqrt(dis_x_2*dis_x_2+dis_y_2*dis_y_2);
                    //virtual_laser_hook.ranges[i]=1.0;
                }
                //
                pub.publish(virtual_laser_hook);
        } else {
            creat_fun=1;
            //
            virtual_laser_hook.header.frame_id=mvibot_seri+"/base_hook";
            virtual_laser_hook.angle_increment=0.01745329;
            virtual_laser_hook.angle_max=M_PI;
            virtual_laser_hook.angle_min=-M_PI;
            virtual_laser_hook.range_min=0;
            virtual_laser_hook.range_max=16.0;
            virtual_laser_hook.scan_time=0.07304524630308151;
            virtual_laser_hook.ranges.resize(360);
            virtual_laser_hook.intensities.resize(360);
            virtual_laser_hook.time_increment=6.372144707711414e-05;
        }
}
void pub_hook_switch(float data){
    static ros::NodeHandle n;
    static ros::Publisher pub_hook_switch = n.advertise<std_msgs::Float32>("/"+mvibot_seri+"/hook_switch",1);
    static ros::Publisher pub_hook_encoder = n.advertise<std_msgs::Float32>("/"+mvibot_seri+"/hook_encoder",1);
    static float creat_fun=0;
        if(creat_fun==1){
                static std_msgs::Float32 msg;
                //
		if(data==0) msg.data=1;
		else msg.data=0;
                //msg.data=data;
                pub_hook_switch.publish(msg);
                //
                msg.data=encoder/180*M_PI;
                pub_hook_encoder.publish(msg);
        }else creat_fun=1;
}
void send_hook_frame(int mode){
    if(mode==1){
        static tf::TransformBroadcaster br;
        static tf::Transform transform;
        static float x,y,theta;
        // send tranfrom for hook base
        transform.setOrigin( tf::Vector3(-d_hook*cos(encoder/180*M_PI), -d_hook*sin(encoder/180*M_PI), 0.0) );
        transform.setRotation(tf::createQuaternionFromYaw(encoder/180*M_PI));
        //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),mvibot_seri+"/base_footprint", mvibot_seri+"/base_hook"));
    }
}