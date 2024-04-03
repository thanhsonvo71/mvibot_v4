#include"../../common/libary/libary_ros.h"
#include"../../common/libary/libary_basic.h"
#include"../../common/set_get_param/set_get_param.h"
#include"../../common/stoi/stoi.h"
using namespace std;
//
extern string mvibot_seri,mode;
extern float ts_scan_sensor;
extern int mvibot_sensor_ready;
extern int radar1_live_status;
extern int radar2_live_status;
extern int camera1_live_status;
extern int camera2_live_status;
extern int battery_live_status;
extern int battery_small_live_status;
extern void pub_history(string data);
//
int start_software_launch=0;
// radar1 status
int radar1_live=0;
float time_live_radar1=0;
// radar2 status
int radar2_live=0;
float time_live_radar2=0;
// camera1 status
int camera1_live=0;
int camera1_config=0;
float time_live_camera1=0;
// camera2 status
int camera2_live=0;
int camera2_config=0;
float time_live_camera2=0;
// uart
int uart_live=0;
// battery status
int battery_status=0;
float time_live_batterry=0;
// battery small status
int battery_small_status=0;
float time_live_batterry_small=0;
// ready sensor when radar 1 2 camera 1 2 is ready
int dym_set_camera1=0,dym_set_camera2=0;
int reset_radar1,reset_radar2,reset_camera1,reset_camera2;
//
void pub_sensor_status(){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub_sensor_status = n.advertise<std_msgs::String>("/"+mvibot_seri+"/sensor_status",1);
    static ros::Publisher pub_sensor_status_string = n1.advertise<std_msgs::String>("/sensor_status_string",100);
    static std_msgs::String msg_sensor_status;
    static float creat_fun=0;
        if(creat_fun==1){
                msg_sensor_status.data=mvibot_seri+"|";
                msg_sensor_status.data=msg_sensor_status.data+"uart:"+to_string((int)uart_live)+"|";
                msg_sensor_status.data=msg_sensor_status.data+"radar1:"+to_string((int)radar1_live)+"|";
                msg_sensor_status.data=msg_sensor_status.data+"radar2:"+to_string((int)radar2_live)+"|";
                msg_sensor_status.data=msg_sensor_status.data+"camera1:"+to_string((int)camera1_live)+"|";
                msg_sensor_status.data=msg_sensor_status.data+"camera2:"+to_string((int)camera2_live)+"|";
                msg_sensor_status.data=msg_sensor_status.data+"battery:"+to_string((int)battery_status)+"|";
                msg_sensor_status.data=msg_sensor_status.data+"battery_small:"+to_string((int)battery_small_status);
                pub_sensor_status.publish(msg_sensor_status);
                pub_sensor_status_string.publish(msg_sensor_status);
        }else creat_fun=1;
}
void reset_sensor(string name){
    static string command;
    command="";
    command=command+"rosnode kill /"+mvibot_seri+"/"+name+" &";
    system(command.c_str());
}
void check_sensor(){
        // radar1 
        if(radar1_live_status==1){
            if(time_live_radar1<0) time_live_radar1=0;
            else time_live_radar1+=(float)ts_scan_sensor;
            if(time_live_radar1>=2.0) time_live_radar1=2.0;
        }else{
            if(time_live_radar1>0) time_live_radar1=0;
            else time_live_radar1-=(float)ts_scan_sensor;
            if(time_live_radar1<=-30.0) time_live_radar1=-30.0;
        }
        radar1_live_status=0;
        if(time_live_radar1>=2.0){
            if(radar1_live!=1) send_history("normal","Radar1 is available");
            radar1_live=1;
        }
        else{
            if(radar1_live==1){
                // time to restartup sensor when sensor is live before
                if(time_live_radar1<=-3.0) 
                {
                    radar1_live=0;
                    time_live_radar1=0;
                    reset_radar1=1;
                    send_history("error","Restart radar1 because no signal");
                }
            }else{
                // time to restartup sensor
                if(time_live_radar1<=-5.0){
                    radar1_live=0;
                    time_live_radar1=0;
                    reset_radar1=1;
                    send_history("error","Restart radar1 because no signal");
                }
            }
        }
        // radar2
        if(radar2_live_status==1){
            if(time_live_radar2<0) time_live_radar2=0;
            else time_live_radar2+=(float)ts_scan_sensor;
            if(time_live_radar2>=2.0) time_live_radar2=2.0;
        }else{
            if(time_live_radar2>0) time_live_radar2=0;
            else time_live_radar2-=(float)ts_scan_sensor;
            if(time_live_radar2<=-30.0) time_live_radar2=-30.0;
        }
        radar2_live_status=0;
        if(time_live_radar2>=2.0){
            if(radar2_live!=1) send_history("normal","Radar2 is available");
            radar2_live=1;
        }
        else{
            if(radar2_live==1){
                // time to restartup sensor when sensor is live before
                if(time_live_radar2<=-3.0) 
                {
                    radar2_live=0;
                    time_live_radar2=0;
                    reset_radar2=1;
                    send_history("error","Restart radar2 because no signal");
                }
            }else{
                // time to restartup sensor
                if(time_live_radar2<=-5.0){
                    radar2_live=0;
                    time_live_radar2=0;
                    reset_radar2=1;
                    send_history("error","Restart radar2 because no signal");
                }
            }
        }
        // camera1
        if(camera1_live_status==1){
            if(time_live_camera1<0) time_live_camera1=0;
            else time_live_camera1+=(float)ts_scan_sensor;
            if(time_live_camera1>=5.0) time_live_camera1=5.0;
        }else{
            if(time_live_camera1>0) time_live_camera1=0;
            else time_live_camera1-=(float)ts_scan_sensor;
            if(time_live_camera1<=-30.0) time_live_camera1=-30.0;
        }
        camera1_live_status=0;
        if(time_live_camera1>=5.0){
            if(camera1_live==0){
                send_history("normal","Camera1 is available");
                camera1_live=1;
                dym_set_camera1=1;
            }
        }
        else{
            if(camera1_live==1){
                // time to restartup sensor when sensor is live before
                if(time_live_camera1<=-5.0) 
                {
                    camera1_live=0;
                    time_live_camera1=0;
                    reset_camera1=1;
                    send_history("error","Restart camera1 because no signal");
                }
            }else{
                // time to restartup sensor
                if(time_live_camera1<=-30.0){
                    camera1_live=0;
                    time_live_camera1=0;
                    reset_camera1=1;
                    send_history("error","Restart camera1 because no signal");
                }
            }
        }
        // camera2
        if(camera2_live_status==1){
            if(time_live_camera2<0) time_live_camera2=0;
            else time_live_camera2+=(float)ts_scan_sensor;
            if(time_live_camera2>=5.0) time_live_camera2=5.0;
        }else{
            if(time_live_camera2>0) time_live_camera2=0;
            else time_live_camera2-=(float)ts_scan_sensor;
            if(time_live_camera2<=-30.0) time_live_camera2=-30.0;
        }
        camera2_live_status=0;
        if(time_live_camera2>=5.0) {
            if(camera2_live==0){
                send_history("normal","Camera2 is available");
                camera2_live=1;
                dym_set_camera2=1;
            }
        }
        else{
            if(camera2_live==1){
                // time to restartup sensor when sensor is live before
                if(time_live_camera2<=-5.0) 
                {
                    camera2_live=0;
                    time_live_camera2=0;
                    reset_camera2=1;
                    send_history("error","Restart camera2 because no signal");
                }
            }else{
                // time to restartup sensor
                if(time_live_camera2<=-30.0){
                    camera2_live=0;
                    time_live_camera2=0;
                    reset_camera2=1;
                    send_history("error","Restart camera2 because no signal");
                }
            }
        }
        if(radar1_live==1 & radar2_live==1 & (camera1_live==1 & camera1_config==1) & (camera2_live==1 & camera2_config==1) & uart_live==1) mvibot_sensor_ready=1;
        else mvibot_sensor_ready=0;
        // first time ready -> start launch mvibot software
        if(start_software_launch==0 & mvibot_sensor_ready==1){
            //
            send_history("normal","Sensor startup success. Start up mode: "+mode);
            //
            static string command;
            start_software_launch=1;
            command="";
            command=command+"roslaunch mvibot_v4 mvibot_software.launch name_seri:="+mvibot_seri+" mode:="+mode+" &";
            system(command.c_str());
        }
        // battery check
        if(battery_live_status==1 & uart_live==1){
            if(time_live_batterry<0) time_live_batterry=0;
            else{
                time_live_batterry+=(float)ts_scan_sensor;
                if(time_live_batterry>=5.0) time_live_batterry=5.0;
            }
        }else{
            if(time_live_batterry>0) time_live_batterry=0;
            else{
                time_live_batterry-=(float)ts_scan_sensor;
                if(time_live_batterry<=-5.0) time_live_batterry=-5.0;
            }
        }
        battery_live_status=0;
        if(time_live_batterry>=3.0){
            if(battery_status!=1) send_history("normal","Battery is aviable");
            battery_status=1;
        }
        if(time_live_batterry<=-2.0){
            if(battery_status!=0) send_history("error","Battery no signal");
            battery_status=0;
        }
        // battery small check
        if(battery_small_live_status==1 & uart_live==1){
            if(time_live_batterry<0) time_live_batterry=0;
            else{
                time_live_batterry+=(float)ts_scan_sensor;
                if(time_live_batterry>=5.0) time_live_batterry=5.0;
            }
        }else{
            if(time_live_batterry>0) time_live_batterry=0;
            else{
                time_live_batterry-=(float)ts_scan_sensor;
                if(time_live_batterry<=-5.0) time_live_batterry=-5.0;
            }
        }
        battery_small_live_status=0;
        if(time_live_batterry>=3.0)
        {
            if(battery_small_status!=1) send_history("normal","Battery small is aviable");
            battery_small_status=1;
        }
        if(time_live_batterry<=-2.0){
            if(battery_small_status!=1) send_history("error","Battery small no signal");
            battery_small_status=0;
        }
        //
        pub_sensor_status();
}   
void action_sensor(){
    static string command;
    command="";
    // check and config camera
    static int n_dym_set_camera1=0,n_dym_set_camera2=0;
    if(dym_set_camera1==1){
        n_dym_set_camera1++;
        if(n_dym_set_camera1>=5){
            //
            static int visual=0,exposure=0,enable_auto_exposure=0;
            visual=-1; exposure=-1; enable_auto_exposure=-1;
            //
            visual=stoi_f(set_get_param("/"+mvibot_seri+"/camera1/camera/stereo_module/set_parameters","visual_preset","int","3"));
            exposure=stoi_f(set_get_param("/"+mvibot_seri+"/camera1/camera/stereo_module/set_parameters","exposure","int","500"));
            enable_auto_exposure=stoi_f(set_get_param("/"+mvibot_seri+"/camera1/camera/stereo_module/set_parameters","enable_auto_exposure","bool","0"));
            dym_set_camera1=0;
            //
            if(visual==3 && exposure==500 && enable_auto_exposure==0){
                camera1_config=1;
                send_history("normal","Camera1 is config available");
            }
            else{
                camera1_live=0;
                time_live_camera1=0;
                reset_camera1=1;
                send_history("error","Restart camera1 because can't config");
            }
        }
    }else n_dym_set_camera1=0;
    if(dym_set_camera2==1){
        n_dym_set_camera2++;
        if(n_dym_set_camera2>=5){
            //
            static int visual=0,exposure=0,enable_auto_exposure=0;
            visual=-1; exposure=-1; enable_auto_exposure=-1;
            //
            visual=stoi_f(set_get_param("/"+mvibot_seri+"/camera2/camera/stereo_module/set_parameters","visual_preset","int","3"));
            exposure=stoi_f(set_get_param("/"+mvibot_seri+"/camera2/camera/stereo_module/set_parameters","exposure","int","500"));
            enable_auto_exposure=stoi_f(set_get_param("/"+mvibot_seri+"/camera2/camera/stereo_module/set_parameters","enable_auto_exposure","bool","0"));
            dym_set_camera2=0;
            //
            if(visual==3 && exposure==500 && enable_auto_exposure==0){
                camera2_config=1;
                send_history("normal","Camera2 is config available");
            }
            else{
                camera2_live=0;
                time_live_camera2=0;
                reset_camera2=1;
                send_history("error","Restart camera2 because can't config");
            }
        }
    }else n_dym_set_camera2=0;
    //
    if(reset_radar1==1){
        reset_sensor("rplidarNode1");
        reset_radar1=0;
    }
    if(reset_radar2==1){
        reset_sensor("rplidarNode2");
        reset_radar2=0;
    }
    if(reset_camera1==1){
        reset_sensor("camera1/camera/realsense2_camera");
        reset_camera1=0;
        camera1_config=0;
    }
    if(reset_camera2==1){   
        reset_sensor("camera2/camera/realsense2_camera");
        reset_camera2=0;
        camera2_config=0;
    }
}
