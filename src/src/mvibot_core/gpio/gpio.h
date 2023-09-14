#include"../../common/libary/libary_basic.h"
#include"../../common/libary/libary_ros.h"
using namespace std;
//seri number
extern string mvibot_seri;
//gpio
extern std_msgs::Float32MultiArray input_user;
extern std_msgs::Float32MultiArray output_user;
//
void pub_output_user_status(){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub_output_user_status = n.advertise<std_msgs::Float32MultiArray>("/"+mvibot_seri+"/output_user_status",1);
    static ros::Publisher pub_output_user_status_string = n.advertise<std_msgs::String>("/output_user_status_string",1);
    //
    static float creat_fun=0;
    static std_msgs::String msg;
        if(creat_fun==1){
                pub_output_user_status.publish(output_user);
                msg.data=mvibot_seri+"|";
                for(int i=0;i<output_user.data.size()-1;i++){
                    msg.data=msg.data+"out"+to_string(i+1)+":"+to_string(output_user.data[i])+"|";
                }
                msg.data=msg.data+"out"+to_string(output_user.data.size())+":"+to_string(output_user.data[output_user.data.size()-1]);
                //pub_output_user_status_string.publish(msg);
        }else creat_fun=1;
}
void pub_output_user_string_status(){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub_output_user_status_string = n.advertise<std_msgs::String>("/output_user_status_string",1);
    static float creat_fun=0;
    static std_msgs::String msg;
        if(creat_fun==1){
                msg.data=mvibot_seri+"|";
                for(int i=0;i<output_user.data.size()-1;i++){
                    msg.data=msg.data+"out"+to_string(i+1)+":"+to_string(output_user.data[i])+"|";
                }
                msg.data=msg.data+"out"+to_string(output_user.data.size())+":"+to_string(output_user.data[output_user.data.size()-1]);
                pub_output_user_status_string.publish(msg);
        }else creat_fun=1;
}
void pub_input_user_status(){
    static ros::NodeHandle n;
    static ros::Publisher pub_input_user_status = n.advertise<std_msgs::Float32MultiArray>("/"+mvibot_seri+"/input_user_status",1);
    static ros::Publisher pub_input_user_status_string = n.advertise<std_msgs::String>("/input_user_status_string",1);
    static float creat_fun=0;
    static std_msgs::String msg;
        if(creat_fun==1){
                pub_input_user_status.publish(input_user);
                msg.data=mvibot_seri+"|";
                for(int i=0;i<input_user.data.size()-1;i++){
                    msg.data=msg.data+"in"+to_string(i+1)+":"+to_string(input_user.data[i])+"|";
                }
                msg.data=msg.data+"in"+to_string(input_user.data.size())+":"+to_string(input_user.data[input_user.data.size()-1]);
                //pub_input_user_status_string.publish(msg);
        }else creat_fun=1;
}
void pub_input_user_string_status(){
    static ros::NodeHandle n;
    static ros::Publisher pub_input_user_status_string = n.advertise<std_msgs::String>("/input_user_status_string",1);
    static float creat_fun=0;
    static std_msgs::String msg;
        if(creat_fun==1){
                msg.data=mvibot_seri+"|";
                for(int i=0;i<input_user.data.size()-1;i++){
                    msg.data=msg.data+"in"+to_string(i+1)+":"+to_string(input_user.data[i])+"|";
                }
                msg.data=msg.data+"in"+to_string(input_user.data.size())+":"+to_string(input_user.data[input_user.data.size()-1]);
                pub_input_user_status_string.publish(msg);
        }else creat_fun=1;
}