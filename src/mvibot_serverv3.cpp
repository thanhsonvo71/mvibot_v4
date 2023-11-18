// must have 
#include "src/common/libary/libary_basic.h"
#include "src/common/thread/thread.h"
#include "src/common/string_Iv2/string_Iv2.h"
//
#include "src/mvibot_server/mvibot_server_init.h"
#include "src/mvibot_server/mvibot_mysql/mvibot_mysql.h"
#include "src/mvibot_server/robot_information/robot_information.h"
#include "src/mvibot_server/layer/layer.h"
#include "src/mvibot_server/map/map.h"
#include "src/mvibot_server/module_gpio_v2/module_gpio_v2.h"
//
using namespace std;
// settime process
long double ts_process1=0.5; //time set for process1
long double ts_process2=1.0; //time set for process2
long double ts_process3=0.05; //time set for process3
long double ts_process4=0.05; //time set for process4
long double ts_process5=0.05; //time set for process5
static pthread_mutex_t process_mutex=PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
static pthread_t p_process1;
static pthread_t p_process2;
static pthread_t p_process3;
static pthread_t p_process4;
static pthread_t p_process5;
// create function system
void lock();
void unlock();
void function1();
void function2();
void function3();
void function4();
void function5();
void time_now(string data);
// creat thread
void * process1(void * nothing){
    void *value_return;
    char name[]="Process A";
	process(name,ts_process1,0,function1);
    return value_return;
}
void * process2(void * nothing){
	char name[]="Process B";
	process(name,ts_process2,1,function2);
    void *value_return;
    return value_return;
}
void * process3(void * nothing){
	char name[]="Process C";
	process(name,ts_process3,2,function3);
    void *value_return;
    return value_return;
}
void * process4(void * nothing){
	char name[]="Process D";
	process(name,ts_process4,3,function4);
    void *value_return;
    return value_return;
}
void * process5(void * nothing){
	char name[]="Process E";
	process(name,ts_process5,4,function5);
    void *value_return;
    return value_return;
}
//
void pub_master_check(){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/master_check",100);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1){
            msg.data="1";
            pub.publish(msg);
    }else creat_fun=1;
}
//
string data_output_set_zip;
void pub_input_zip(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/input_user_status_string_zip",100);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1){
            msg.data=data;
            pub.publish(msg);
    }else creat_fun=1;
}
void pub_output_zip(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/output_user_status_string_zip",100);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1){
            msg.data=data;
            pub.publish(msg);
    }else creat_fun=1;
}
void pub_output_set_zip(string data){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/output_user_set_string_zip",100);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1){
            msg.data=data;
            pub.publish(msg);
    }else creat_fun=1;
}
void pub_zip(){
    static string data;
    //input
    data="";
    for(int i=0;i<my_robots.size();i++){
        data=data+"<"+my_robots[i].input_user_status+">";
    }
    for(int i=0;i<my_module_gpio_v2s.size();i++){
        //data=data+"<"+my_module_gpio_v2s[i].my_node->input_user_status_string+">";
    }
    pub_input_zip(data);
    // output
    data="";
    for(int i=0;i<my_robots.size();i++){
        data=data+"<"+my_robots[i].output_user_status+">";
    }
    for(int i=0;i<my_module_gpio_v2s.size();i++){
        //data=data+"<"+my_module_gpio_v2s[i].my_node->output_user_status_string+">";
    }
    pub_output_zip(data);   
    // output_set
    if(data_output_set_zip!=""){
        pub_output_set_zip(data_output_set_zip);
        data_output_set_zip="";
    }
}
//
void IAMf(const std_msgs::String& msg){
	lock();
      static int is_have=0;
        is_have=0;
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        if(data.data1.size()==2){
            for(int i=0;i<my_robots.size();i++){
                if(my_robots[i].name_seri==data.data1[0]) {
                    is_have=1;
                    my_robots[i].time_out=0;
                }
            }
            if(is_have==0){
                my_robots.resize(my_robots.size()+1);
                my_robots[my_robots.size()-1].name_seri=data.data1[0];
                my_robots[my_robots.size()-1].type=data.data1[1];
                my_robots[my_robots.size()-1].update_database=0;
                my_robots[my_robots.size()-1].node=new node_v2_3;
                my_robots[my_robots.size()-1].node->name_seri=my_robots[my_robots.size()-1].name_seri;   
                my_robots[my_robots.size()-1].node->init();
                printf("Add new: %s \n" , my_robots[my_robots.size()-1].name_seri.c_str());
            }
        }       
    unlock();
}
void robot_statusf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        for(int i=0;i<my_robots.size();i++){
            if(my_robots[i].name_seri==data.data1[0]){
                my_robots[i].robot_status=msg.data;
                break;
            }
        }
    unlock();  
}
void sensor_statusf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        for(int i=0;i<my_robots.size();i++){
            if(my_robots[i].name_seri==data.data1[0]){
                my_robots[i].sensor_status=msg.data;
                break;
            }
        }
    unlock();
}
void battery_statusf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        for(int i=0;i<my_robots.size();i++){
            if(my_robots[i].name_seri==data.data1[0]){
                my_robots[i].battery_status=msg.data;
                break;
            }
        }
    unlock();
}
void battery_small_status_stringf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        for(int i=0;i<my_robots.size();i++){
            if(my_robots[i].name_seri==data.data1[0]){
                my_robots[i].battery_small_status=msg.data;
                break;
            }
        }
    unlock();
}
void battery_cell_statusf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        for(int i=0;i<my_robots.size();i++){
            if(my_robots[i].name_seri==data.data1[0]){
                my_robots[i].battery_cell_status=msg.data;
                break;
            }
        }
    unlock();  
}
void motor_right_statusf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        for(int i=0;i<my_robots.size();i++){
            if(my_robots[i].name_seri==data.data1[0]){
                my_robots[i].motor_right_status=msg.data;
                break;
            }
        }
    unlock();
}
void motor_left_statusf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        for(int i=0;i<my_robots.size();i++){
            if(my_robots[i].name_seri==data.data1[0]){
                my_robots[i].motor_left_status=msg.data;
                break;
            }
        }
    unlock();
}
void input_user_statusf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        for(int i=0;i<my_robots.size();i++){
            if(my_robots[i].name_seri==data.data1[0]){
                my_robots[i].input_user_status=msg.data;
                break;
            }
        }
    unlock();
}
void output_user_statusf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        for(int i=0;i<my_robots.size();i++){
            if(my_robots[i].name_seri==data.data1[0]){
                my_robots[i].output_user_status=msg.data;
                break;
            }
        }
    unlock();
}
void robot_config_status_stringf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        for(int i=0;i<my_robots.size();i++){
            if(my_robots[i].name_seri==data.data1[0]){
                my_robots[i].robot_config_status=msg.data;
                break;
            }
        }
    unlock();
}
void mapf(const nav_msgs::OccupancyGrid & msg){
    lock();
        my_map=msg;
        // for(int i=0;i<my_map.data.size();i++) my_map.data[i]=-1;
    unlock();
}
void request_mapf(const std_msgs::String & msg){
    lock();
        request_map_topic=1;
    unlock();
}
void robot_tff(const std_msgs::String & msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        send_tranfrom(stof(data.data1[1]),stof(data.data1[2]),stof(data.data1[3]),stof(data.data1[4]),"/map","/"+data.data1[0]+"/base_footprint");
    unlock();
}
void request_actionf(const std_msgs::String & msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        if(data.data1.size()>=2){
            if(data.data1[0]=="save_map"){
                static string_Iv2 data2;
                data2.detect(data.data1[1],"~","=","~");
                // 
                static string name_map,name_robot;
                name_map="";
                name_robot="";
                for(int i=0;i<data2.data1.size();i++){
                    if(data2.data1[i]=="name_map") name_map=data2.data2[i];
                    if(data2.data1[i]=="name_robot") name_robot=data2.data2[i];
                }
                if(name_map!="" & name_robot!=""){
                    static string cmd;
                    cmd="";
                    cmd=cmd+"rosrun map_server map_saver map:="+name_robot+"/map"+" -f /home/mvibot/interface_mvibot_v2/maps/"+name_map+"&";
                    system(cmd.c_str());
                }
            }
            if(data.data1[0]=="delete_map"){
                static string_Iv2 data2;
                data2.detect(data.data1[1],"~","=","~");
                // 
                static string name_map;
                name_map="";
                for(int i=0;i<data2.data1.size();i++){
                    if(data2.data1[i]=="name_map") name_map=data2.data2[i];
                }
                if(name_map!=""){
                    static string cmd;
                    cmd="";
                    cmd=cmd+"rm /home/mvibot/interface_mvibot_v2/maps/"+name_map+".*";
                    system(cmd.c_str());
                }
            }
            if(data.data1[0]=="active_map"){
                static string_Iv2 data2;
                data2.detect(data.data1[1],"~","=","~");
                // 
                static string name_map;
                name_map="";
                for(int i=0;i<data2.data1.size();i++){
                    if(data2.data1[i]=="name_map") name_map=data2.data2[i];
                }
                if(name_map!=""){
                    static string cmd;
                    cmd="";
                    cmd=cmd+"echo "+name_map+" > "+"/home/mvibot/catkin_ws/src/mvibot/config/map";
                    system(cmd.c_str());
                }
            }
        }
    unlock();
}
void pub_map_select(){
    static ros::NodeHandle n;
    static ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>( "/map_selector", 1 );
    static float creat_fun=0;
	if(creat_fun==1)
	{
        map_pub.publish(map_selector);   
	} else creat_fun=1;
}
void map_selectf(const std_msgs::String & msg){
    map_selector=get_map_select("/home/mvibot/interface_mvibot_v2/maps/"+msg.data);
    pub_map_select(); 
}
void output_user_set_stringf(const std_msgs::String & msg){
   lock();
        static string_Iv2 data;
        data.detect(msg.data,"","||","");
        if(data.data1.size()==2){
            for(int i=0;i<my_module_gpio_v2s.size();i++){
                if(data.data1[0]==my_module_gpio_v2s[i].my_node->name_node){
                    cout<<msg.data<<endl;
                    my_module_gpio_v2s[i].my_node->output_user_set_string=msg.data;
                }
            }
        }
        //
        data_output_set_zip=data_output_set_zip+"<"+msg.data+">";
   unlock();
}
void reset_serverf(const std_msgs::String & msg){
   lock();
        if(msg.data=="1"){
            string cmd;
            cmd="rosnode kill /tf2_web_republisher &";
            system(cmd.c_str());
            cmd="rosnode kill /rosbridge_server &";
            system(cmd.c_str());
        }
   unlock();
}
int main(int argc, char** argv){
    //
    ts_my_robots=(float)ts_process1;
    try
    {
	    std::ifstream file("/home/mvibot/catkin_ws/src/mvibot/config/map");
	    std::string str; 
	    std::string data;
	    data="";
	    while (std::getline(file, str))
	    {
            // Process str
            data=data+str;
	    }
        name_map_active=data;
        cout<<name_map_active<<endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    //
    data_base_init();
    table_init();
    //
    ros::init(argc, argv, "mvibot_serverv3");
    get_robots_frist();
    //
    pub_input_user_status_string("");
    pub_output_user_status_string("");
    pub_output_set_zip("");
    static int res;
    res=pthread_create(&p_process1,NULL,process1,NULL);
    res=pthread_create(&p_process2,NULL,process2,NULL);
    //
    ros::NodeHandle n0,n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17;
    ros::Subscriber sub0 = n0.subscribe("/IAM", 100, IAMf);    
    ros::Subscriber sub1 = n1.subscribe("/robot_status_string", 100, robot_statusf);
    // sensor 
    ros::Subscriber sub2 = n2.subscribe("/sensor_status_string", 100, sensor_statusf);
    // battery
    ros::Subscriber sub3 = n3.subscribe("/battery_status_string", 100, battery_statusf);
    ros::Subscriber sub17 = n17.subscribe("/battery_small_status_string", 100, battery_small_status_stringf);
    ros::Subscriber sub4 = n4.subscribe("/battery_cell_status_string", 100, battery_cell_statusf);
    // motor
    ros::Subscriber sub5 = n5.subscribe("/motor_right_status_string", 100, motor_right_statusf);
    ros::Subscriber sub6 = n6.subscribe("/motor_left_status_string", 100, motor_left_statusf);
    // I/O
    ros::Subscriber sub7 = n7.subscribe("/input_user_status_string", 100, input_user_statusf);
    ros::Subscriber sub8 = n8.subscribe("/output_user_status_string", 100, output_user_statusf);
    // status config robot
    ros::Subscriber sub9 = n9.subscribe("/robot_config_status_string", 100, robot_config_status_stringf);
    //
    ros::Subscriber sub10 = n10.subscribe("/map", 1, mapf);   
    ros::Subscriber sub11 = n11.subscribe("/request_map", 1, request_mapf);
    //
    ros::Subscriber sub12 = n12.subscribe("/request_action", 1, request_actionf);
    ros::Subscriber sub13 = n13.subscribe("/map_select", 1, map_selectf);    
    //
    ros::Subscriber sub14 = n14.subscribe("/output_user_set_string", 100, output_user_set_stringf);
    ros::Subscriber sub15 = n15.subscribe("/reset_server", 1, reset_serverf); 
    ros::Subscriber sub16 = n16.subscribe("/robot_tf", 1000, robot_tff); 
    //
    while (ros::ok())
    {
        ros::spinOnce();
        usleep(1e5);
    }
    return 0;
}
//
void lock(){
    pthread_mutex_lock(&process_mutex);
}
void unlock(){
    pthread_mutex_unlock(&process_mutex);
}
void time_now(string name){
    lock();
        static struct timespec realtime;
        clock_gettime(CLOCK_REALTIME, &realtime);
        cout<<name+"|Time:"<<std::fixed << std::setprecision(5)<<((long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9)<<endl;
    unlock();
}
void function1(){
    lock();
        database_process();
        layer_process();
        pub_map();
        pub_master_check();
    unlock();
}
void function2(){
    lock();
        pub_zip();
        
    unlock();
}
void function3(){
      
}
void function4(){  
}
void function5(){
}