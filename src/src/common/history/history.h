#include"../libary/libary_ros.h"
#include"../libary/libary_basic.h"
using namespace std;
extern string mvibot_seri;
void pub_history(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/history",100);
    static std_msgs::String msg;
    static float creat_fun=0;
    if(creat_fun==1){
        msg.data=data;
        pub.publish(msg);
    }else creat_fun=1;
}
void send_history(string mode,string data){
    string msg;
    msg="/type>"+mode+"/";
    msg=msg+"/data>"+data+"/";
    pub_history(msg);
}