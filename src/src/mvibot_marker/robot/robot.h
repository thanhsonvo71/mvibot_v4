#include "../mvibot_marker_init.h"
#include "../../common/libary/libary_ros.h"
#include "../../common/string_Iv2/string_Iv2.h"
#include "../../common/stof/stof.h"
#include "../../common/set_get_param/set_get_param.h"
using namespace  std;
//
void process_data(string data){
    static string_Iv2 data_I;
    data_I.detect(data,"~","=","~");
    safe_x1=0.01; safe_x2=0.01; safe_y1=0.01; safe_y2=0.01;
    //
    my_marker.marker_dir="";
    my_marker.marker_type="";
    for(int i=0;i<data_I.data1.size();i++){
        //
        if(data_I.data1[i]=="marker_type")      my_marker.marker_type=data_I.data2[i];
        if(data_I.data1[i]=="marker_dir")       my_marker.marker_dir=data_I.data2[i];
        if(data_I.data1[i]=="off_set_x1")       my_marker.off_set_x=stod_f(data_I.data2[i]);
        if(data_I.data1[i]=="off_set_y1")       my_marker.off_set_y=stod_f(data_I.data2[i]);
        if(data_I.data1[i]=="off_set_dis")      my_marker.off_set_dis=stod_f(data_I.data2[i]);
        if(data_I.data1[i]=="off_set_angle")    my_marker.off_set_angle=stod_f(data_I.data2[i]);
        //
        if(data_I.data1[i]=="sx1") safe_x1=stof_f(data_I.data2[i]);
        if(data_I.data1[i]=="sx2") safe_x2=stof_f(data_I.data2[i]);
        if(data_I.data1[i]=="sy1") safe_y1=stof_f(data_I.data2[i]);
        if(data_I.data1[i]=="sy2") safe_y2=stof_f(data_I.data2[i]);
    }
    my_marker.marker_data=data;
}
void pub_cmd_vel(float v,float w){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<geometry_msgs::Twist>("/"+mvibot_seri+"/cmd_vel", 1);
    static geometry_msgs::Twist cmd_msg;
    static float creat_fun=0;
	if(creat_fun==1)
	{
        //
        if(start!=1){
            v=0;
            w=0;
        } 
        cmd_msg.linear.x=(double)v;
        cmd_msg.angular.z=(double)w;
		//
        pub.publish(cmd_msg);
	} else creat_fun=1;
}
void robot_emg(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/robot_emg", 1);
	static float creat_fun=0;
    std_msgs::String msg;
    msg.data="1";
	if(creat_fun==1)
	{
		pub.publish(msg);
	} else creat_fun=1;
}
int check_safe(){
    static int free_space;
    free_space=0;
    for(int i=0;i<scan_safe.ranges.size();i++){
        static float x,y,theta;
        theta=scan_safe.angle_min+i*scan_safe.angle_increment;
        x=scan_safe.ranges[i]*cos(theta);
        y=scan_safe.ranges[i]*sin(theta);
        //check safe
        if(x>=x1_footprint-safe_x1 & x<=x2_footprint+safe_x2){
            if(y>=y1_footprint-safe_y1 & y<=y2_footprint+safe_y2){
                if(!((x>x1_footprint & x<x2_footprint) & (y>y1_footprint & y<y2_footprint))){
                     cout<<"OB"<<endl;
                     cout<<x<<"|"<<y<<endl;
                     free_space=1;
                     break;
                }
            }
        }
    }
    return free_space;
}
int get_footprint(){
    static string footprint_return;
    footprint_return=set_get_param("/"+mvibot_seri+"/move_base_flex/global_costmap/set_parameters","footprint","string","");
    cout<<"footprint:"<<footprint_return<<endl;
    if(footprint_return!="-1"){
        static string string_1;
        string_1="";
        for(int i=0;i<footprint_return.length();i++){
            if(footprint_return[i]!='[' & footprint_return[i]!=']') 
            string_1=string_1+footprint_return[i];
        }
        static string_Iv2 string_2;
        string_2.detect(string_1,"",",","");
        for(int i=0;i<string_2.data1.size();i=i+2){
            if(stof_f(string_2.data1[i]) >0) x2_footprint=stof_f(string_2.data1[i]);
            if(stof_f(string_2.data1[i]) <0) x1_footprint=stof_f(string_2.data1[i]);
        }
        for(int i=1;i<string_2.data1.size();i=i+2){
            if(stof_f(string_2.data1[i]) >0) y2_footprint=stof_f(string_2.data1[i]);
            if(stof_f(string_2.data1[i]) <0) y1_footprint=stof_f(string_2.data1[i]);
        }
        cout<<x1_footprint<<"|"<<y1_footprint<<"|"<<x2_footprint<<"|"<<y2_footprint<<endl;
        return 1;
    }else return 0;
}
void pub_status_marker(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/status_marker", 1);
	static float creat_fun=0;
	if(creat_fun==1)
	{
        static std_msgs::String data;
        if(start==0) data.data="0";
        if(start==1) data.data="1";
        if(start==2) data.data="2";
		pub.publish(data);
	} else creat_fun=1;
}
void pub_status(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/status_marker_test", 1);
	static float creat_fun=0;
	if(creat_fun==1)
	{
        static std_msgs::String data;
        data.data=to_string(my_marker.status)+"|"+to_string(my_marker.active_step);
		pub.publish(data);
	} else creat_fun=1;
}