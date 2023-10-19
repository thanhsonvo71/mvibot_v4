#include "src/common/libary/libary_basic.h"
#include "src/common/libary/libary_ros.h"
#include "src/common/thread/thread.h"
#include "src/common/string_Iv2/string_Iv2.h"
#include "src/common/get_position/get_position.h"
//
using namespace  std;
string mvibot_seri;
string system_command;
// socket var 
#define PORT 9082
int socket_port[1000];
int server_fd;
struct sockaddr_in address; 
int opt = 1; 
int addrlen = sizeof(address); 
int n_client=0;
char str_cli_ip[INET_ADDRSTRLEN];
struct sockaddr_in* ip_client;
class port_receive{
    public:
        string ip;
        int port;
};
port_receive port_clinet_receive[1000];
int max_n_clinet=1000;
static string server_topic=""; //_localhost
//
static char start_char='!';
static char end_char='?';
//
static vector<string> msg_to_socket;
static vector<string> msg_to_action;
static int enable_get_laser_data=0;
static int enable_output_user_set_string=0;
//
int get_map=0;
int get_map2=0;
int request_map=0;
int request_map2=0;
// thread var
static pthread_mutex_t process_mutex=PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;	
static pthread_t p_process1;
static pthread_t p_process2;
static pthread_t p_process3;
static pthread_t p_process4;
static pthread_t p_process5;
static pthread_t p_process6;
static long double ts1=0.05;
static long double ts2=0.05;
static long double ts3=0.05;
static long double ts4=0.05;
static long double ts5=1.0;
static long double ts6=0.2;
//
void time_now(string name);
void lock();
void unlock();
void modify_socket();
void send_data_socket_server();
void read_data_socket_server();
void function2();
void function3();
void function4();
void function5();
void function6();
void process_data(string data);
//
void time_now(string name){
    lock();
        static struct timespec realtime;
        clock_gettime(CLOCK_REALTIME, &realtime);
        cout<<name+"|Time:"<<std::fixed << std::setprecision(5)<<((long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9)<<endl;
    unlock();
}
void * process1(void * nothing){
    while(1){
        while(n_client<max_n_clinet){
            //listen
            if (listen(server_fd, max_n_clinet) < 0) 
            { 
                perror("listen"); 
                exit(EXIT_FAILURE); 
            } 
            //accept 
            if ((socket_port[n_client] = accept(server_fd, (struct sockaddr *)&address,(socklen_t*)&addrlen))<0) 
            { 
                perror("accept"); 
                exit(EXIT_FAILURE); 
            }
            sleep(1);
            pthread_mutex_lock(&process_mutex); 
                ip_client= (struct sockaddr_in*)&address;
                inet_ntop(AF_INET, &ip_client->sin_addr, str_cli_ip, INET_ADDRSTRLEN);
                //
                port_clinet_receive[n_client].ip=str_cli_ip;
                port_clinet_receive[n_client].port=ntohs(ip_client->sin_port);
                n_client++;
                cout<<"accept: "<<n_client<<endl;
                //
            pthread_mutex_unlock(&process_mutex); 
        }
        usleep(1000000);
    }  
    void *value_return;
    return value_return;
}
void * process2(void * nothing){
    char name[]="Process 2";
    process(name,ts2,0,function2);
    void *value_return;
    return value_return; 
}
void * process3(void * nothing){
    char name[]="Process 3";
    process(name,ts3,1,function3);
    void *value_return;
    return value_return;
}
void * process4(void * nothing){
    char name[]="Process 4";
    process(name,ts4,2,function4);
    void *value_return;
    return value_return;
}
void * process5(void * nothing){
    char name[]="Process 5";
    process(name,ts5,3,function5);
    void *value_return;
    return value_return;
}
void * process6(void * nothing){
    char name[]="Process 4";
    process(name,ts6,4,function6);
    void *value_return;
    return value_return;
}
//
nav_msgs::OccupancyGrid  my_map;
nav_msgs::OccupancyGrid  my_map2;
void pub_map(){
    static ros::NodeHandle n;
    static ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>( "/map"+server_topic, 1 );
    static float creat_fun=0;
	if(creat_fun==1)
	{
        if(get_map==1){
            static uint32_t n_subtopic=0;
            static int have_to_pub=0;
            my_map.info.origin.orientation.w=0;
            my_map.info.origin.orientation.w=1;
            have_to_pub=0;
            if(map_pub.getNumSubscribers()!=n_subtopic){
                if(n_subtopic< map_pub.getNumSubscribers()) have_to_pub=1;
                n_subtopic=map_pub.getNumSubscribers();
            }
            if(request_map==1){
                have_to_pub=1;
                request_map=0;
            }
            if(have_to_pub) map_pub.publish(my_map);
        }
    }else creat_fun=1;
}
void pub_map2(){
    static ros::NodeHandle n;
    static ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>( "/map2"+server_topic, 1 );
    static float creat_fun=0;
	if(creat_fun==1)
	{
        if(get_map2==1){
            static uint32_t n_subtopic=0;
            static int have_to_pub=0;
            my_map2.info.origin.orientation.w=0;
            my_map2.info.origin.orientation.w=1;
            have_to_pub=0;
            if(map_pub.getNumSubscribers()!=n_subtopic){
                if(n_subtopic< map_pub.getNumSubscribers()) have_to_pub=1;
                n_subtopic=map_pub.getNumSubscribers();
            }
            if(request_map2==1){
                have_to_pub=1;
                request_map2=0;
            }
            if(have_to_pub) map_pub.publish(my_map2);
        }
    }else creat_fun=1;
}
void pub_cmd_vel(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<geometry_msgs::Twist>("/"+mvibot_seri+"/cmd_vel", 1);
    static float creat_fun=0;
    static geometry_msgs::Twist msg;
    if(creat_fun==1)
    {
           static string_Iv2 data1;
           data1.detect(data,"","|","");
           msg.linear.x=stof(data1.data1[0]);
           msg.angular.z=stof(data1.data1[1]);
           pub.publish(msg);
    } else {
        creat_fun=1;
    }

}
void pub_set_config(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/set_config", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_music_start(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::Float32>("/"+mvibot_seri+"/music_start", 1);
    static float creat_fun=0;
    static std_msgs::Float32 msg;
    if(creat_fun==1)
    {
           msg.data=stof(data);
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_music_name(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/music_name", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_initialpose_web(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/initialpose_web", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_output_user_set(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/output_user_set", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_robot_shutdown(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/robot_shutdown", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_master_check(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/master_check", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_mission_normal(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/mission_normal", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_mission_error(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/mission_error", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_mission_battery(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/mission_charge_battery", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_mission_action(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/mission_action", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_output_user_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/output_user_status_string", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_input_user_status_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/input_user_status_string", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}
void pub_output_user_set_string(string data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/output_user_set_string", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
           msg.data=data;
           pub.publish(msg);
    } else {
        creat_fun=1;
    }
}

//
void IAMf(const std_msgs::String& msg){
    lock();
        //
        if(n_client > 0){
            static string_Iv2 data;
            data.detect(msg.data,"","|","");
            if(data.data1.size()>=2){
                if(data.data1[0]==mvibot_seri){
                    static string string_process;
                    string_process="";
                    string_process=start_char;
                    string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
                    string_process=string_process+"{[(name_topic/=IAM)]}";
                    string_process=string_process+"{[(data/="+msg.data+")]}";
                    string_process=string_process+end_char;
                    //
                    msg_to_socket.resize(msg_to_socket.size()+1);
                    msg_to_socket[msg_to_socket.size()-1]=string_process;
                }
            }
        }
    unlock();
}
void request_mapf(const std_msgs::String& msg){
    lock();
        if(msg.data=="1"){
            request_map=1;
            request_map2=1;
        }
    unlock();
}
void sensor_status_stringf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=sensor_status_string)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
        }
    unlock();
}
void robot_status_stringf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=robot_status_string)]}";
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
        if(n_client > 0){
            static string_Iv2 data;
            data.detect(msg.data,"","|","");
            if(data.data1.size()>=2){
                if(data.data1[0]==mvibot_seri){
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
            }
        }
    unlock();
}
void input_user_status_stringf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string_Iv2 data;
            data.detect(msg.data,"","|","");
            if(data.data1.size()>=2){
                if(data.data1[0]==mvibot_seri){
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
            }
        }
    unlock();
}
void battery_cell_status_stringf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string_Iv2 data;
            data.detect(msg.data,"","|","");
            if(data.data1.size()>=2){
                if(data.data1[0]==mvibot_seri){
                    static string string_process;
                    string_process="";
                    string_process=start_char;
                    string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
                    string_process=string_process+"{[(name_topic/=battery_cell_status_string)]}";
                    string_process=string_process+"{[(data/="+msg.data+")]}";
                    string_process=string_process+end_char;
                    //
                    msg_to_socket.resize(msg_to_socket.size()+1);
                    msg_to_socket[msg_to_socket.size()-1]=string_process;
                }
            }
        }
    unlock();
}
void battery_small_status_stringf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string_Iv2 data;
            data.detect(msg.data,"","|","");
            if(data.data1.size()>=2){
                if(data.data1[0]==mvibot_seri){
                    static string string_process;
                    string_process="";
                    string_process=start_char;
                    string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
                    string_process=string_process+"{[(name_topic/=battery_small_status_string)]}";
                    string_process=string_process+"{[(data/="+msg.data+")]}";
                    string_process=string_process+end_char;
                    //
                    msg_to_socket.resize(msg_to_socket.size()+1);
                    msg_to_socket[msg_to_socket.size()-1]=string_process;
                }
            }
        }
    unlock();
}
void battery_status_stringf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string_Iv2 data;
            data.detect(msg.data,"","|","");
            if(data.data1.size()>=2){
                if(data.data1[0]==mvibot_seri){
                    static string string_process;
                    string_process="";
                    string_process=start_char;
                    string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
                    string_process=string_process+"{[(name_topic/=battery_status_string)]}";
                    string_process=string_process+"{[(data/="+msg.data+")]}";
                    string_process=string_process+end_char;
                    //
                    msg_to_socket.resize(msg_to_socket.size()+1);
                    msg_to_socket[msg_to_socket.size()-1]=string_process;
                }
            }
        }
    unlock();
}
void robot_config_status_stringf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string_Iv2 data;
            data.detect(msg.data,"","|","");
            if(data.data1.size()>=2){
                if(data.data1[0]==mvibot_seri){
                    static string string_process;
                    string_process="";
                    string_process=start_char;
                    string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
                    string_process=string_process+"{[(name_topic/=robot_config_status_string)]}";
                    string_process=string_process+"{[(data/="+msg.data+")]}";
                    string_process=string_process+end_char;
                    //
                    msg_to_socket.resize(msg_to_socket.size()+1);
                    msg_to_socket[msg_to_socket.size()-1]=string_process;
                }
            }
        }
    unlock();
}
void motor_left_status_stringf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string_Iv2 data;
            data.detect(msg.data,"","|","");
            if(data.data1.size()>=2){
                if(data.data1[0]==mvibot_seri){
                    static string string_process;
                    string_process="";
                    string_process=start_char;
                    string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
                    string_process=string_process+"{[(name_topic/=motor_left_status_string)]}";
                    string_process=string_process+"{[(data/="+msg.data+")]}";
                    string_process=string_process+end_char;
                    //
                    msg_to_socket.resize(msg_to_socket.size()+1);
                    msg_to_socket[msg_to_socket.size()-1]=string_process;
                }
            }
        }
    unlock();
}
void motor_right_status_stringf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string_Iv2 data;
            data.detect(msg.data,"","|","");
            if(data.data1.size()>=2){
                if(data.data1[0]==mvibot_seri){
                    static string string_process;
                    string_process="";
                    string_process=start_char;
                    string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
                    string_process=string_process+"{[(name_topic/=motor_right_status_string)]}";
                    string_process=string_process+"{[(data/="+msg.data+")]}";
                    string_process=string_process+end_char;
                    //
                    msg_to_socket.resize(msg_to_socket.size()+1);
                    msg_to_socket[msg_to_socket.size()-1]=string_process;
                }
            }
        }
    unlock();
}
void output_user_set_stringf(const std_msgs::String& msg){
    lock();
        if(n_client > 0 & enable_output_user_set_string==1){
            static string_Iv2 data;
            data.detect(msg.data,"","||","");
            if(data.data1[0]!=mvibot_seri){
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
                enable_output_user_set_string=0;
            }
        }
    unlock();
}
void hook_switchf(const std_msgs::Float32& msg){
    lock();
        if(n_client > 0){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=hook_switch)]}";
            string_process=string_process+"{[(data/="+to_string(msg.data)+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
        }
    unlock();
}
void hook_encoderf(const std_msgs::Float32& msg){
    lock();
        if(n_client > 0){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=hook_encoder)]}";
            string_process=string_process+"{[(data/="+to_string(msg.data)+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
        }
    unlock();
}
void scanf(const sensor_msgs::LaserScan& msg){
    lock();
        if(enable_get_laser_data==1 & n_client > 0){
            static string string_process;
            //
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=laser_scan)]}";
            string_process=string_process+"{[(data/=";
            for(int i=0;i<msg.ranges.size();i++){
                string_process=string_process+to_string(msg.ranges[i])+";";
            }
            string_process=string_process+")]}"+end_char;
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
            enable_get_laser_data=0;
        }
    unlock();
}
void robot_list_wifif(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=robot_list_wifi)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
        }
    unlock();
}
void local_variablef(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=local_variable)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
        }
    unlock();
}
void mission_action_inforf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=mission_action_infor)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
        }
    unlock();
}
void mission_memoryf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=mission_memory)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
        }
    unlock();
}
void footprintf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=footprint)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
        }
    unlock();
}
void pathf(const std_msgs::String& msg){
    lock();
        if(n_client > 0){
            static string string_process;
            string_process="";
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=path)]}";
            string_process=string_process+"{[(data/="+msg.data+")]}";
            string_process=string_process+end_char;
            //
            msg_to_socket.resize(msg_to_socket.size()+1);
            msg_to_socket[msg_to_socket.size()-1]=string_process;
        }
    unlock();
}
int main(int argc, char** argv) 
{ 
    //
    ros::init(argc, argv, "mvibot_transport");   // creat mvibot_node
    ros::NodeHandle nh("~");
    nh.getParam("mvibot_seri", mvibot_seri);
    //
    pub_map();
    pub_map2();
    //
    pub_cmd_vel("");
    pub_set_config("");
    pub_music_start("");
    pub_music_name("");
    pub_initialpose_web("");
    pub_output_user_set("");
    pub_robot_shutdown("");
    pub_master_check("");
    //
    pub_mission_normal("");
    pub_mission_battery("");
    pub_mission_error("");
    pub_mission_action("");
    pub_input_user_status_string("");
    pub_output_user_status_string("");
    pub_output_user_set_string("");
    modify_socket();
    //
    int res;
    res=pthread_create(&p_process1,NULL,process1,NULL);
    res=pthread_create(&p_process2,NULL,process2,NULL);
    res=pthread_create(&p_process3,NULL,process3,NULL);
    res=pthread_create(&p_process4,NULL,process4,NULL);
    res=pthread_create(&p_process5,NULL,process5,NULL);
    res=pthread_create(&p_process6,NULL,process6,NULL);
    //
    ros::NodeHandle n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17,n18,n19,n20,n21,n22;
    ros::Subscriber sub1 = n1.subscribe("/IAM"+server_topic, 1, IAMf);
    ros::Subscriber sub2 = n2.subscribe("/request_map"+server_topic, 1, request_mapf);
    ros::Subscriber sub3 = n3.subscribe("/sensor_status_string"+server_topic, 1, sensor_status_stringf);
    ros::Subscriber sub4 = n4.subscribe("/robot_status_string"+server_topic, 1, robot_status_stringf);
    ros::Subscriber sub5 = n5.subscribe("/output_user_status_string"+server_topic, 1, output_user_status_stringf);
    ros::Subscriber sub6 = n6.subscribe("/input_user_status_string"+server_topic, 1, input_user_status_stringf);
    ros::Subscriber sub7 = n7.subscribe("/battery_cell_status_string"+server_topic, 1, battery_cell_status_stringf);
    ros::Subscriber sub8 = n8.subscribe("/battery_small_status_string"+server_topic, 1, battery_small_status_stringf);
    ros::Subscriber sub9 = n9.subscribe("/battery_status_string"+server_topic, 1, battery_status_stringf);
    //
    ros::Subscriber sub10 = n10.subscribe("/"+mvibot_seri+"/hook_switch", 1, hook_switchf);
    ros::Subscriber sub11 = n11.subscribe("/"+mvibot_seri+"/hook_encoder", 1, hook_encoderf);
    //
    ros::Subscriber sub12 = n12.subscribe("/"+mvibot_seri+"/laser/scan", 1, scanf);
    //
    ros::Subscriber sub13 = n13.subscribe("/robot_config_status_string", 1, robot_config_status_stringf);
    ros::Subscriber sub14 = n14.subscribe("/motor_left_status_string", 1, motor_left_status_stringf);
    ros::Subscriber sub15 = n15.subscribe("/motor_right_status_string", 1, motor_right_status_stringf);
    ros::Subscriber sub16 = n16.subscribe("/output_user_set_string", 1, output_user_set_stringf);
    ros::Subscriber sub17 = n17.subscribe("/"+mvibot_seri+"/robot_list_wifi", 1, robot_list_wifif);
    //
    ros::Subscriber sub18 = n18.subscribe("/"+mvibot_seri+"/local_variable", 10, local_variablef);
    ros::Subscriber sub19 = n19.subscribe("/"+mvibot_seri+"/mission_action_infor", 10, mission_action_inforf);
    ros::Subscriber sub20 = n20.subscribe("/"+mvibot_seri+"/mission_memory", 10, mission_memoryf);
    ros::Subscriber sub21 = n21.subscribe("/"+mvibot_seri+"/footprint", 1, footprintf);
    ros::Subscriber sub22 = n22.subscribe("/"+mvibot_seri+"/path", 1, pathf);
    ros::spin();
    return 0; 
} 
void lock(){
    pthread_mutex_lock(&process_mutex);
}
void unlock(){
    pthread_mutex_unlock(&process_mutex);
}
void modify_socket(){
    A:
    // creat file socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
    { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    }  
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;
    // add for socket
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, (const char*)&tv, sizeof tv)) //&opt, sizeof(opt))) 
    { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT );  
    // bind ip
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) 
    { 
        perror("bind failed"); 
        close(server_fd);
        usleep(1e5);
        goto A;
        exit(EXIT_FAILURE); 
    }
}
void send_data_socket_server(){
    lock();
        if(n_client>0){
            static string data_send_to_socket;
            data_send_to_socket="";
            for(int i=0;i<msg_to_socket.size();i++){
                data_send_to_socket+=msg_to_socket[i];
            }
            msg_to_socket.resize(0);
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
                send(socket_port[n_client-1], byte_to_socket.c_str() ,byte_to_socket.length(), MSG_NOSIGNAL );
            }
	        
	    }
    unlock();
}
void read_data_socket_server(){
    if(n_client>0){
        //
        static long num_byte_limit=1000000;
        static int num_byte;
        static char data_socket[1000000]={};
        static int count;
        static int data_error;
        static int i;
        //
        memset(data_socket, 0, sizeof data_socket); 
        i=n_client-1;
        if(n_client>0) num_byte=read( socket_port[i], data_socket, 1000000);
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
                        cout<<"finish"<<endl;
                        output=string_return;
                        collect_data=0;
                        string_return="";
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
}
void process_data(string data){
    if(data[0]=='*'){
        //
        static string_Iv2 data_Iv2;
        for(int i=0;i<data.length();i++){
            if(data[i]=='*'){
                static string data1;
                data1="";
                for(int j=i+1;j<data.length();j++){
                    if(data[j]!='*') data1+=data[j];
                    else break;
                }
                data_Iv2.detect(data1,"{[(",":",")]}");
                break;
            }
        }
        //
        static vector<int8_t> map_data;
        map_data.resize(0);
        for(int k=1;k<data.length();k++){
            if(data[k]=='*'){
                static string value;
                value="";
                for(int j=k+1;j<data.length();j++){
                    if(data[1]=='*'){
                        k=j;
                        break;
                    }
                    if(data[j]!=';') value+=data[j];
                    else{
                        // cout<<value<<endl;
                        map_data.resize(map_data.size()+1);
                        map_data[map_data.size()-1]=stoi(value);
                        value="";
                    }
                }
                break;
            }
        }
        data_Iv2.print();
        cout<<map_data.size()<<endl;
        for(int i=0;i<data_Iv2.data1.size();i++){
            if(data_Iv2.data1[i]=="topic_name" & data_Iv2.data2[i]=="map"){
                my_map.data=map_data;
                for(int j=0;j<data_Iv2.data1.size();j++){
                    if(data_Iv2.data1[j]=="frame_id")           my_map.header.frame_id=data_Iv2.data2[j];
                    if(data_Iv2.data1[j]=="resolution")         my_map.info.resolution=stof(data_Iv2.data2[j]);
                    if(data_Iv2.data1[j]=="width")              my_map.info.width=stoi(data_Iv2.data2[j]);
                    if(data_Iv2.data1[j]=="height")             my_map.info.height=stoi(data_Iv2.data2[j]);
                    if(data_Iv2.data1[j]=="ox")                 my_map.info.origin.position.x=stof(data_Iv2.data2[j]);
                    if(data_Iv2.data1[j]=="oy")                 my_map.info.origin.position.y=stof(data_Iv2.data2[j]);
                }
                cout<<my_map.info.width<<endl;
                cout<<my_map.info.height<<endl;
                get_map=1;
                break;
            }
            if(data_Iv2.data1[i]=="topic_name" & data_Iv2.data2[i]=="map2"){
                my_map2.data=map_data;
                for(int j=0;j<data_Iv2.data1.size();j++){
                    if(data_Iv2.data1[j]=="frame_id")           my_map2.header.frame_id=data_Iv2.data2[j];
                    if(data_Iv2.data1[j]=="resolution")         my_map2.info.resolution=stof(data_Iv2.data2[j]);
                    if(data_Iv2.data1[j]=="width")              my_map2.info.width=stoi(data_Iv2.data2[j]);
                    if(data_Iv2.data1[j]=="height")             my_map2.info.height=stoi(data_Iv2.data2[j]);
                    if(data_Iv2.data1[j]=="ox")                 my_map2.info.origin.position.x=stof(data_Iv2.data2[j]);
                    if(data_Iv2.data1[j]=="oy")                 my_map2.info.origin.position.y=stof(data_Iv2.data2[j]);
                }
                cout<<my_map2.info.width<<endl;
                cout<<my_map2.info.height<<endl;
                get_map2=1;
                break;
            }
        }

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
                if(name_topic=="cmd_vel")           pub_cmd_vel(data2);
                if(name_topic=="set_config")        pub_set_config(data2);
                if(name_topic=="music_start")       pub_music_start(data2);
                if(name_topic=="music_name")        pub_music_name(data2);
                if(name_topic=="initialpose_web")   pub_initialpose_web(data2);
                if(name_topic=="output_user_set")   pub_output_user_set(data2);
                if(name_topic=="robot_shutdown")    pub_robot_shutdown(data2);
                if(name_topic=="master_check")      pub_master_check(data2);
                if(name_topic=="mission_normal")    pub_mission_normal(data2);
                if(name_topic=="mission_battery")   pub_mission_battery(data2);
                if(name_topic=="mission_error")     pub_mission_error(data2);
                if(name_topic=="mission_action")    pub_mission_action(data2);
                if(name_topic=="input_user_status_string")     pub_input_user_status_string(data2);
                if(name_topic=="output_user_status_string")    pub_output_user_status_string(data2);
                if(name_topic=="output_user_set_string")       pub_output_user_set_string(data2);
            }
        }
    }
    //
}
void function2(){
	// don't set delay big than time_set in this function,timer set min 0.001 s
        lock();
            // printf("Action function 2\n"); 
            // time_now("At:");
            for(int i=0;i<msg_to_action.size();i++){
                process_data(msg_to_action[i]);
            }
            msg_to_action.resize(0);
        unlock();
        // communicate with stm
        // process data
}
void function3(){
    // don't set delay big than in this function,timer set min 0.001 s
        lock();
            // printf("Action function 3\n"); 
            // time_now("At:");
        unlock();
        // read data socket
        read_data_socket_server();
}
void function4(){
    // don't set delay big than in this function,timer set min 0.001 s
        lock();
            // printf("Action function 4\n"); 
            // time_now("At:");
        unlock();
        // send data socket
        send_data_socket_server();
}
void function5(){
    // don't set delay big than in this function,timer set min 0.001 s
    lock();
            // printf("Action function 5\n"); 
            // time_now("At:");
            pub_map();
            pub_map2();
    unlock();
}
void function6(){
    // don't set delay big than in this function,timer set min 0.001 s
            // printf("Action function 6\n"); 
            // time_now("At:");
    if(n_client>0){
        static float *position;
        position=get_position("/map","/"+mvibot_seri+"/base_footprint");
        if(position[0]!=-1 &  position[1]!=-1 | position[2]!=-1 | position[3]!=-1){
            static string string_process;
            string_process=start_char;
            string_process=string_process+"{[(name_seri/="+mvibot_seri+")]}";
            string_process=string_process+"{[(name_topic/=tf)]}";
            string_process=string_process+"{[(data/=";
            string_process=string_process+to_string(position[0])+"|"+to_string(position[1])+"|"+to_string(position[2])+"|"+to_string(position[3]);
            string_process=string_process+")]}"+end_char;
            //
            lock();
                msg_to_socket.resize(msg_to_socket.size()+1);
                msg_to_socket[msg_to_socket.size()-1]=string_process;
            unlock();
        }
        lock();
            enable_get_laser_data=1;
            enable_output_user_set_string=1;
        unlock();
    }      
}