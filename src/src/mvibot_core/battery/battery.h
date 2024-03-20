#include"../../common/libary/libary_basic.h"
#include"../../common/libary/libary_ros.h"
//#include"../mvibot_core_init.h"
using namespace std;
extern string mvibot_seri;
// battery 
extern float battery_soc;
extern float battery_vol;
extern float battery_cycle;
extern float battery_mah_now;
extern float battery_mah_max;
extern float battery_guard;
extern float battery_temperature;
extern float battery_num_cell;
extern float battery_current;
extern float battery_cell[8];
extern int battery1_charge;
// small battery
extern float battery_small_soc;
extern float battery_small_vol;
extern float low_battery_small;
extern float battery_small_temperature;
extern float battery_small_current;
extern float battery_small_mah_now;
extern float battery_small_mah_max;
extern float battery_small_num_cell;
extern int battery2_charge;
//
extern int battery_live_status;
extern int battery_small_live_status; 
extern float distance_robot,distance_wheel_left,distance_wheel_right;
//
void pub_battery_status(){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub_battery_status = n.advertise<std_msgs::String>("/"+mvibot_seri+"/battery_status",1);
    static ros::Publisher pub_battery_status_string = n1.advertise<std_msgs::String>("/battery_status_string",1);
    static float creat_fun=0;
        if(creat_fun==1){
                static std_msgs::String msg;
                msg.data=mvibot_seri+"|";
                if(battery_live_status==1){
                    msg.data=msg.data+"soc:"+to_string((int)battery_soc)+"|";
                    msg.data=msg.data+"vol:"+to_string(battery_vol)+"|";
                    msg.data=msg.data+"cycle:"+to_string((int)battery_cycle)+"|";
                    msg.data=msg.data+"capacity_now:"+to_string(battery_mah_now/100)+"|";
                    msg.data=msg.data+"capacity_max:"+to_string(battery_mah_max/100)+"|";
                    if(battery_current>=0) msg.data=msg.data+"charge:1|";
                    else msg.data=msg.data+"charge:0|";
                    msg.data=msg.data+"current:"+to_string(battery_current/100)+"|";
                    msg.data=msg.data+"num_cell:"+to_string((int)battery_num_cell)+"|";
                    msg.data=msg.data+"temperature:"+to_string(battery_temperature);
                }else msg.data="N/A";
                pub_battery_status.publish(msg);
                pub_battery_status_string.publish(msg);
                //
                static string cmd;
                cmd="echo $(date +'%d/%m/%Y %H:%M:%S') robot:"+to_string(distance_robot/1000)+" wheel_left:"+to_string(distance_wheel_left/1000)+" wheel_right:"+to_string(distance_wheel_right/1000);
                cmd=cmd+" \""+msg.data+"\" >> /home/mvibot/catkin_ws/src/mvibot_v4/history/battery.log";
                system(cmd.c_str());
                //
        }else {
            static string cmd;
            cmd="echo $(date +'%d/%m/%Y %H:%M:%S') start log > /home/mvibot/catkin_ws/src/mvibot_v4/history/battery.log";
            system(cmd.c_str());
            creat_fun=1;
        }
}
void pub_battery_cell_status(){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub_battery_cell_status = n.advertise<std_msgs::String>("/"+mvibot_seri+"/battery_cell_status",1);
    static ros::Publisher pub_battery_cell_status_string = n1.advertise<std_msgs::String>("/battery_cell_status_string",1);
    static float creat_fun=0;
        if(creat_fun==1){
                static std_msgs::String msg;
                 msg.data=mvibot_seri+"|";
                if(battery_live_status==1){
                    for(int i=0;i<7;i++){
                        msg.data=msg.data+"Cell"+to_string(i+1)+":"+to_string(battery_cell[i])+"|";
                    }
                    msg.data=msg.data+"Cell"+to_string(7+1)+":"+to_string(battery_cell[7]);
                }else msg.data="N/A";
                pub_battery_cell_status.publish(msg);
                pub_battery_cell_status_string.publish(msg);
        }else creat_fun=1;
}
void pub_battery_small_status(){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub_battery_status = n.advertise<std_msgs::String>("/"+mvibot_seri+"/battery_small_status",1);
    static ros::Publisher pub_battery_status_string = n1.advertise<std_msgs::String>("/battery_small_status_string",1);
    static float creat_fun=0;
        if(creat_fun==1){
                static std_msgs::String msg;
                msg.data=mvibot_seri+"|";
                if(battery_small_live_status==1){
                    msg.data=msg.data+"soc:"+to_string((int)battery_small_soc)+"|";
                    msg.data=msg.data+"vol:"+to_string(battery_small_vol)+"|";
                    msg.data=msg.data+"capacity_now:"+to_string(battery_small_mah_now/100)+"|";
                    msg.data=msg.data+"capacity_max:"+to_string(battery_small_mah_max/100)+"|";
                    if(battery_current>=0) msg.data=msg.data+"charge:1|";
                    else msg.data=msg.data+"charge:0|";
                    msg.data=msg.data+"current:"+to_string(battery_small_current/100)+"|";
                    msg.data=msg.data+"num_cell:"+to_string((int)battery_small_num_cell)+"|";
                    msg.data=msg.data+"temperature:"+to_string(battery_small_temperature);
                }else msg.data="N/A";
                pub_battery_status.publish(msg);
                pub_battery_status_string.publish(msg);
        }else creat_fun=1;
}                    
