#include "src/common/libary/libary_basic.h"
#include "src/common/libary/libary_ros.h"
#include "src/common/thread/thread.h"
#include "src/common/string_Iv2/string_Iv2.h"
#include "src/common/get_position/get_position.h"
#include "src/common/send_tranfrom/send_tranfrom.h"
//
using namespace  std;
string mvibot_seri;
// socket var
#define PORT 9082
struct sockaddr_in address; 
int sock = 0; 
struct sockaddr_in serv_addr; 
char add[225]="127.0.0.1";
//
static char start_char='!';
static char end_char='?';
static vector<string> msg_to_socket;
static vector<string> msg_to_action;
void process_data(string data);
// thread var
static pthread_mutex_t process_mutex=PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;	
static pthread_t p_process1;
static pthread_t p_process2;
static pthread_t p_process3;
static pthread_t p_process4;
static long double ts1=3.0;
static long double ts2=0.05;
static long double ts3=0.05;
static long double ts4=0.05;
// return from command
void lock();
void unlock();
void time_now(string name);
int modify_socket();
void function1();
void function2();
void function3();
void function4();
// thread 
void * process1(void * nothing){
    char name[]="Process 1";
    process(name,ts1,0,function1);
    void *value_return;
    return value_return;
}
void * process2(void * nothing){
    char name[]="Process 2";
    process(name,ts2,1,function2);
    void *value_return;
    return value_return;
}
void * process3(void * nothing){
    char name[]="Process 3";
    process(name,ts3,2,function3);
    void *value_return;
    return value_return;
}
void * process4(void * nothing){
    char name[]="Process 4";
    process(name,ts4,3,function4);
    void *value_return;
    return value_return;
}
//
int get_map=0;
int get_map2=0;
vector<string> add_vector_string(vector<string> data){
    vector<string> data1;
    data1.resize(0);
    for(int i=0;i<data.size();i=i+2){
        if(i+1 >= data.size()){
            data1.resize(data1.size()+1);
            data1[data1.size()-1]=data[i];
        }else{
            data1.resize(data1.size()+1);
            data1[data1.size()-1]=data[i]+data[i+1];
        }
    }
    if(data1.size()<=1) return data1;
    else return add_vector_string(data1);
}
void mapf(const nav_msgs::OccupancyGrid & msg){
    if(get_map==0 & msg.data.size() !=0){
            //
            static string process_string;
            //
            process_string="";
            process_string=start_char;
            process_string=process_string+"*{[(topic_name:map)]}";
            process_string=process_string+"{[(frame_id:"+msg.header.frame_id+")]}";
            process_string=process_string+"{[(ox:"+to_string(msg.info.origin.position.x)+")]}";
            process_string=process_string+"{[(oy:"+to_string(msg.info.origin.position.y)+")]}";
            process_string=process_string+"{[(resolution:"+to_string(msg.info.resolution)+")]}";
            process_string=process_string+"{[(width:"+to_string(msg.info.width)+")]}";
            process_string=process_string+"{[(height:"+to_string(msg.info.height)+")]}";
            process_string=process_string+"*";
            //
            // cout<<msg.data.size()<<endl;
            static vector<string> data_zip;
            data_zip.resize(0);
            data_zip.resize(1);
            for(int i=0;i<msg.data.size();i++){
                if(data_zip[data_zip.size()-1].length()<10000){
                    data_zip[data_zip.size()-1]=data_zip[data_zip.size()-1]+to_string(msg.data[i])+";";
                }else{
                    data_zip.resize(data_zip.size()+1);
                    data_zip[data_zip.size()-1]=data_zip[data_zip.size()-1]+to_string(msg.data[i])+";";
                }
            }
            process_string=process_string+add_vector_string(data_zip)[0]+"*"+end_char;
            data_zip.resize(0);
            cout<<"get_map"<<endl;
            //
        lock();
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=process_string;
            process_string="";
        unlock();
        get_map=1;
    }
}
void map2f(const nav_msgs::OccupancyGrid & msg){
    if(get_map2==0 & msg.data.size() !=0){
        //
        static string process_string;
        //
            process_string="";
            process_string=start_char;
            process_string=process_string+"*{[(topic_name:map2)]}";
            process_string=process_string+"{[(frame_id:"+msg.header.frame_id+")]}";
            process_string=process_string+"{[(ox:"+to_string(msg.info.origin.position.x)+")]}";
            process_string=process_string+"{[(oy:"+to_string(msg.info.origin.position.y)+")]}";
            process_string=process_string+"{[(resolution:"+to_string(msg.info.resolution)+")]}";
            process_string=process_string+"{[(width:"+to_string(msg.info.width)+")]}";
            process_string=process_string+"{[(height:"+to_string(msg.info.height)+")]}";
            process_string=process_string+"*";
            //
            cout<<msg.data.size()<<endl;
            static vector<string> data_zip;
            data_zip.resize(0);
            data_zip.resize(1);
            //msg.data.size()
            for(int i=0;i<msg.data.size();i++){
                if(data_zip[data_zip.size()-1].length()<10000){
                    data_zip[data_zip.size()-1]=data_zip[data_zip.size()-1]+to_string(msg.data[i])+";";
                }else{
                    data_zip.resize(data_zip.size()+1);
                    data_zip[data_zip.size()-1]=data_zip[data_zip.size()-1]+to_string(msg.data[i])+";";
                }
            }
            process_string=process_string+add_vector_string(data_zip)[0]+"*"+end_char;
            //
            cout<<"get_map2"<<endl;
            data_zip.resize(0);
        lock();
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=process_string;
            process_string="";
        unlock();
        get_map2=1;
    }
}
//
void pub_requestmap(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/request_map", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
            pub.publish(msg);
    } else {
        creat_fun=1;
        msg.data="1";
    }
}
void pub_IAM(string data){
    static ros::NodeHandle n;
    static ros::Publisher iam_pub = n.advertise<std_msgs::String>("/IAM",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            iam_pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_sensor_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher sensor_status = n.advertise<std_msgs::String>("/sensor_status_string",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            sensor_status.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_robot_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/robot_status_string",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_laser_scan(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("/"+mvibot_seri+"/laser/scan",1);
    static float creat_fun=0;
    static sensor_msgs::LaserScan virtual_laser;
    if(creat_fun==1){
            virtual_laser.header.stamp=ros_timenow();         
            static string_Iv2 data_Iv2;
            data_Iv2.detect(data,"",";","");
            virtual_laser.ranges.resize(data_Iv2.data1.size()-1);
            for(int i=0;i<data_Iv2.data1.size()-1;i++){
                   if(data_Iv2.data1[i]!="nan") virtual_laser.ranges[i]=stof(data_Iv2.data1[i]);
                   else virtual_laser.ranges[i]=17;
            }
            pub.publish(virtual_laser);
    }else {
        creat_fun=1;
        virtual_laser.header.frame_id=mvibot_seri+"/base_footprint";
        virtual_laser.angle_increment=0.005454154219478369;
        virtual_laser.angle_max=3.1415927410125732;
        virtual_laser.angle_min=-3.1415927410125732;
        virtual_laser.range_min=0;
        virtual_laser.range_max=16.0;
        virtual_laser.scan_time=0.07304524630308151;
        virtual_laser.intensities.resize(0);
        virtual_laser.time_increment=0.0;
    }
}
void pub_tf(string data){
    static string_Iv2 data1;
    //data1.detect(data,"","|","");
    //send_tranfrom(stof(data1.data1[0]),stof(data1.data1[1]),stof(data1.data1[2]),stof(data1.data1[3]),"/map","/"+mvibot_seri+"/base_footprint");
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/robot_tf",1000);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=mvibot_seri+"|"+data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_output_user_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/output_user_status_string",100);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_input_user_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/input_user_status_string",100);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_battery_cell_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/battery_cell_status_string",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_battery_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/battery_status_string",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_battery_small_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/battery_small_status_string",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_robot_config_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/robot_config_status_string",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_motor_left_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/motor_left_status_string",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_motor_right_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/motor_right_status_string",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_output_user_set_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/output_user_set_string",100);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_hook_encoder(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::Float32>("/"+mvibot_seri+"/hook_encoder",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::Float32 msg;
            msg.data=stof(data);
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_hook_switch(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::Float32>("/"+mvibot_seri+"/hook_switch",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::Float32 msg;
            msg.data=stof(data);
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_robot_list_wifi(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/robot_list_wifi",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_local_variable(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/local_variable",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_mission_action_infor(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/mission_action_infor",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_mission_memory(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/mission_memory",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_footprint(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/footprint",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
void pub_path(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/path",1);
    static float creat_fun=0;
    if(creat_fun==1){
            static std_msgs::String msg;
            msg.data=data;
            pub.publish(msg);
    }else {
        creat_fun=1;
    }
}
//
void cmd_velf(const geometry_msgs::Twist& msg){
    lock();
            msg.linear.x;
            msg.angular.z;
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=cmd_vel)]}";
            string_process=string_process+"{[(data/="+to_string(msg.linear.x)+"|"+to_string(msg.angular.z)+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
    unlock();
}
void set_configf(const std_msgs::String& msg){
    lock();
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=set_config)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
    unlock();
}
void music_startf(const std_msgs::Float32& msg){
    lock();
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=music_start)]}";
            string_process=string_process+"{[(data/="+to_string((int)msg.data)+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
    unlock();
}
void music_namef(const std_msgs::String& msg){
    lock();
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=music_name)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
    unlock();
}
void initialpose_webf(const std_msgs::String& msg){
    lock();
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=initialpose_web)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
    unlock();
}
void output_user_setf(const std_msgs::String& msg){
    lock();
        static string string_process;
        string_process="";
        string_process=start_char;
        string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
        string_process=string_process+"{[(name_topic/=output_user_set)]}";
        string_process=string_process+"{[(data/="+msg.data+")]}";
        string_process=string_process+end_char;
        //
        msg_to_socket.resize(msg_to_socket.size()+1);
        msg_to_socket[msg_to_socket.size()-1]=string_process;    
    unlock();
}
void robot_shutdownf(const std_msgs::String& msg){
    lock();
        static string string_process;
        string_process="";
        string_process=start_char;
        string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
        string_process=string_process+"{[(name_topic/=robot_shutdown)]}";
        string_process=string_process+"{[(data/="+msg.data+")]}";
        string_process=string_process+end_char;
        //
        msg_to_socket.resize(msg_to_socket.size()+1);
        msg_to_socket[msg_to_socket.size()-1]=string_process;    
    unlock();
}
void output_user_set_stringf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","||","");
        if(data.data1[0]==mvibot_seri){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=output_user_set_string)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;  
        }    
    unlock();
}
void master_checkf(const std_msgs::String& msg){
    lock();
        static string string_process;
        string_process="";
        string_process=start_char;
        string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
        string_process=string_process+"{[(name_topic/=master_check)]}";
        string_process=string_process+"{[(data/="+msg.data+")]}";
        string_process=string_process+end_char;
        //
        msg_to_socket.resize(msg_to_socket.size()+1);
        msg_to_socket[msg_to_socket.size()-1]=string_process;    
    unlock();
}
void mission_normalf(const std_msgs::String& msg){
    lock();
        static string string_process;
        string_process="";
        string_process=start_char;
        string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
        string_process=string_process+"{[(name_topic/=mission_normal)]}";
        string_process=string_process+"{[(data/="+msg.data+")]}";
        string_process=string_process+end_char;
        //
        msg_to_socket.resize(msg_to_socket.size()+1);
        msg_to_socket[msg_to_socket.size()-1]=string_process;    
    unlock();
}
void mission_errorf(const std_msgs::String& msg){
    lock();
        static string string_process;
        string_process="";
        string_process=start_char;
        string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
        string_process=string_process+"{[(name_topic/=mission_error)]}";
        string_process=string_process+"{[(data/="+msg.data+")]}";
        string_process=string_process+end_char;
        //
        msg_to_socket.resize(msg_to_socket.size()+1);
        msg_to_socket[msg_to_socket.size()-1]=string_process;    
    unlock();
}
void mission_batteryf(const std_msgs::String& msg){
    lock();
        static string string_process;
        string_process="";
        string_process=start_char;
        string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
        string_process=string_process+"{[(name_topic/=mission_battery)]}";
        string_process=string_process+"{[(data/="+msg.data+")]}";
        string_process=string_process+end_char;
        //
        msg_to_socket.resize(msg_to_socket.size()+1);
        msg_to_socket[msg_to_socket.size()-1]=string_process;    
    unlock();
}
void mission_actionf(const std_msgs::String& msg){
    lock();
        static string string_process;
        string_process="";
        string_process=start_char;
        string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
        string_process=string_process+"{[(name_topic/=mission_action)]}";
        string_process=string_process+"{[(data/="+msg.data+")]}";
        string_process=string_process+end_char;
        //
        msg_to_socket.resize(msg_to_socket.size()+1);
        msg_to_socket[msg_to_socket.size()-1]=string_process;    
    unlock();
}
void input_user_status_stringf(const std_msgs::String& msg){
    lock();
    static string_Iv2 data;
    data.detect(msg.data,"","|","");
    if(data.data1[0]!=mvibot_seri){
        static string string_process;
        string_process="";
        string_process=start_char;
        string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
        string_process=string_process+"{[(name_topic/=input_user_status_string)]}";
        string_process=string_process+"{[(data/="+msg.data+")]}";
        string_process=string_process+end_char;
        //
        msg_to_socket.resize(msg_to_socket.size()+1);
        msg_to_socket[msg_to_socket.size()-1]=string_process;    
    }
    unlock();
}
void output_user_status_stringf(const std_msgs::String& msg){
    lock();
    static string_Iv2 data;
    data.detect(msg.data,"","|","");
    if(data.data1[0]!=mvibot_seri){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=output_user_status_string)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;    
    }
    unlock();
}
// main
int main(int argc, char** argv) 
{ 
    //
    ros::init(argc, argv, "mvibot_master");   // creat mvibot_node
    ros::NodeHandle nh("~");
    nh.getParam("mvibot_seri", mvibot_seri);
    //
    pub_IAM("");
    pub_requestmap("");
    pub_sensor_status_string("");
    pub_robot_status_string("");
    pub_battery_cell_status_string ("");
    pub_battery_status_string ("");
    pub_battery_small_status_string ("");
    pub_laser_scan("");
    pub_motor_left_status_string("");
    pub_motor_right_status_string("");
    pub_output_user_status_string("");
    pub_input_user_status_string("");
    pub_robot_config_status_string("");
    pub_output_user_set_string("");
    pub_hook_encoder("");
    pub_hook_switch("");
    pub_robot_list_wifi("");
    pub_local_variable("");
    pub_mission_action_infor("");
    pub_mission_memory("");
    pub_footprint("");
    pub_path("");
    //
    while(modify_socket()==-1){
        sleep(2);
    }
    int res;
    res=pthread_create(&p_process1,NULL,process1,NULL);
    res=pthread_create(&p_process2,NULL,process2,NULL);
    res=pthread_create(&p_process3,NULL,process3,NULL);
    res=pthread_create(&p_process4,NULL,process4,NULL);
    //
    ros::NodeHandle n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17,n18,n19,n20,n21;
    ros::Subscriber sub1 = n1.subscribe("/map", 1, mapf);
    ros::Subscriber sub2 = n2.subscribe("/map2", 1, map2f);
    //
    ros::Subscriber sub3 = n3.subscribe("/"+mvibot_seri+"/cmd_vel", 1, cmd_velf);
    ros::Subscriber sub4 = n4.subscribe("/"+mvibot_seri+"/set_config", 1, set_configf);
    ros::Subscriber sub5 = n5.subscribe("/"+mvibot_seri+"/music_start", 1, music_startf);
    ros::Subscriber sub6 = n6.subscribe("/"+mvibot_seri+"/music_name", 1, music_namef);
    ros::Subscriber sub7 = n7.subscribe("/"+mvibot_seri+"/initialpose_web", 1, initialpose_webf);
    ros::Subscriber sub8 = n8.subscribe("/"+mvibot_seri+"/output_user_set", 1, output_user_setf);
    ros::Subscriber sub9 = n9.subscribe("/"+mvibot_seri+"/robot_shutdown", 1, robot_shutdownf);
    // //
    ros::Subscriber sub10 = n10.subscribe("/"+mvibot_seri+"/mission_normal", 1, mission_normalf);
    ros::Subscriber sub11 = n11.subscribe("/"+mvibot_seri+"/mission_error", 1, mission_errorf);
    ros::Subscriber sub12 = n12.subscribe("/"+mvibot_seri+"/mission_battery", 1, mission_batteryf);
    ros::Subscriber sub13 = n13.subscribe("/"+mvibot_seri+"/mission_action", 1, mission_actionf);
    // //
    ros::Subscriber sub14 = n14.subscribe("/output_user_set_string", 100, output_user_set_stringf);
    ros::Subscriber sub15 = n15.subscribe("/input_user_status_string", 100, input_user_status_stringf);
    ros::Subscriber sub16 = n16.subscribe("/output_user_status_string", 100, output_user_status_stringf);
    ros::Subscriber sub17 = n17.subscribe("/master_check", 1, master_checkf);   
    ros::spin();
    return 0; 
} 
void time_now(string name){
     static struct timespec realtime;
     clock_gettime(CLOCK_REALTIME, &realtime);
     cout<<name+"|Time:"<<std::fixed << std::setprecision(5)<<((long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9)<<endl;
}
void lock(){
    pthread_mutex_lock(&process_mutex);
}
void unlock(){
    pthread_mutex_unlock(&process_mutex);
}
int modify_socket(){
    static int value_return;
    value_return=1;
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        printf("\n Socket creation error \n"); 
        return -1; 
    } 
    memset(&serv_addr, '0', sizeof(serv_addr)); 
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(PORT); 
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, add, &serv_addr.sin_addr) <= 0)  
    { 
        printf("\nInvalid address/ Address not supported \n"); 
        return -1; 
    } 
    // connect
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("\nConnection Failed \n"); 
        return -1; 
    } 
    return value_return;
}
void read_data_socket_server(){
        //
        static long num_byte_limit=1000000;
        static int num_byte;
        static char data_socket[1000000]={};
        static int count;
        static int data_error;
        //
        memset(data_socket,'\0',sizeof data_socket);
        num_byte=read( sock , data_socket, num_byte_limit);
        //
        static string output;
        output="";
        if(num_byte>0){
            static string string_return;
            static int collect_data;
            for(int i=0;i<num_byte;i++){
                if(collect_data==1){
                    if(data_socket[i]==end_char){
                        cout<<"Data get:\t"<<string_return<<endl;
                        output=string_return;
                        collect_data=0;
                        string_return="";
                        //
                        lock();
                            if(output!=""){
                                msg_to_action.resize(msg_to_action.size()+1);
                                msg_to_action[msg_to_action.size()-1]=output;
                            }
                        unlock();
                    }else string_return+=data_socket[i];
                }else{
                    if(data_socket[i]==start_char){
                        collect_data=1;
                    }else string_return="";
                }
            }
        }
}
void send_data_socket_server(){
    lock();
            static string data_send_to_socket;
            data_send_to_socket="";
            for(int i=0;i<msg_to_socket.size();i++){
                data_send_to_socket+=msg_to_socket[i];
            }
            msg_to_socket.resize(0);
            vector<string>().swap(msg_to_socket);
            //
            static long num_byte_limit=1000000;
            static string memory_msg="";
            static string byte_to_socket="";
            memory_msg+=data_send_to_socket;
            if(memory_msg.length()<num_byte_limit){
                byte_to_socket=memory_msg;
                memory_msg="";
            }else{
                byte_to_socket=memory_msg.substr(0,num_byte_limit);
                memory_msg=memory_msg.substr(num_byte_limit,memory_msg.length());
            }
            if(byte_to_socket.length()!=0){
                cout<<"Data send:\t"<<byte_to_socket<<endl;
                send(sock, byte_to_socket.c_str() ,byte_to_socket.length(), MSG_NOSIGNAL );
            }
	       
    unlock();
}
void process_data(string data){
    if(data[0]=='*'){
        // use for topic map
    }else{
        static string_Iv2 data_Iv2;
        data_Iv2.detect(data,"{[(","/=",")]}");
        //data_Iv2.print();
        //
        static string name_seri,name_topic,data2;
        name_seri=""; name_topic=""; data2="";
        //
        for(int i=0;i<data_Iv2.data1.size();i++){
            if(data_Iv2.data1[i]=="name_seri")  name_seri=data_Iv2.data2[i];
            if(data_Iv2.data1[i]=="name_topic") name_topic=data_Iv2.data2[i];
            if(data_Iv2.data1[i]=="data")       data2=data_Iv2.data2[i];
        }
        //
        if(name_seri!="" & name_topic !="" & data!=""){
            if(name_seri==mvibot_seri){
                if(name_topic=="IAM")                         pub_IAM(data2);
                if(name_topic=="sensor_status_string")        pub_sensor_status_string(data2);
                if(name_topic=="robot_status_string")         pub_robot_status_string(data2);
                if(name_topic=="laser_scan")                  pub_laser_scan(data2);
                if(name_topic=="tf")                          pub_tf(data2);
                if(name_topic=="input_user_status_string")    pub_input_user_status_string(data2);
                if(name_topic=="output_user_status_string")   pub_output_user_status_string(data2);
                if(name_topic=="battery_cell_status_string")  pub_battery_cell_status_string(data2);
                if(name_topic=="battery_status_string")       pub_battery_status_string(data2);
                if(name_topic=="battery_small_status_string") pub_battery_small_status_string(data2);
                if(name_topic=="robot_config_status_string")  pub_robot_config_status_string(data2);
                if(name_topic=="motor_right_status_string")   pub_motor_right_status_string(data2);
                if(name_topic=="motor_left_status_string")    pub_motor_left_status_string(data2);
                if(name_topic=="output_user_set_string")      pub_output_user_set_string(data2);
                if(name_topic=="hook_encoder")                pub_hook_encoder(data2);
                if(name_topic=="hook_switch")                 pub_hook_switch(data2);
                if(name_topic=="robot_list_wifi")             pub_robot_list_wifi(data2);
                if(name_topic=="local_variable")              pub_local_variable(data2);
                if(name_topic=="mission_action_infor")        pub_mission_action_infor(data2);
                if(name_topic=="mission_memory")              pub_mission_memory(data2);
                if(name_topic=="footprint")                   pub_footprint(data2);
                if(name_topic=="path")                        pub_path(data2);
            }
        }
    }
    //
}
void function1(){
    // don't set delay bigger than timeset in this function,timer set min 0.001 s
        lock();
            // printf("Action function 1\n"); 
            // time_now("At:");
            if(get_map ==0 | get_map2==0) pub_requestmap("");
        unlock();
}
void function2(){
        // don't set delay bigger than timeset in this function,timer set min 0.001 s

        //
        lock();
            // printf("Action function 2\n"); 
            // time_now("At:");
        unlock();
        send_data_socket_server();

}
void function3() {
    // don't set  delay bigger than timeset in this function,timer set min 0.001 s
        lock();
            // printf("Action function 3\n"); 
            // time_now("At:");
        unlock();
        // read data from socket
        read_data_socket_server();
}
void function4() {
    // don't set  delay bigger than timeset in this function,timer set min 0.001 s
        lock();
            // printf("Action function 4\n"); 
            // time_now("At:");
            for(int i=0;i<msg_to_action.size();i++){
                // cout<<msg_to_action[i]<<endl;
                process_data(msg_to_action[i]);
            }
            msg_to_action.resize(0);
            vector<string>().swap(msg_to_action);
        unlock();
}




