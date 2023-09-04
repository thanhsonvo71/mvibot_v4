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
#define PORT 9083
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
nav_msgs::OccupancyGrid  my_map;
void pub_map(int mode){
    static ros::NodeHandle n;
    static ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>( "/"+mvibot_seri+"/map", 1 );
    static float creat_fun=0;
	if(creat_fun==1)
	{
        my_map.info.origin.orientation.w=0;
        my_map.info.origin.orientation.w=1;
        if(mode==1) map_pub.publish(my_map);
        else{
            static uint32_t n_subtopic=0;
            static int have_to_pub=0;
            have_to_pub=0;
            if(map_pub.getNumSubscribers()!=n_subtopic){
                if(n_subtopic< map_pub.getNumSubscribers()) have_to_pub=1;
                n_subtopic=map_pub.getNumSubscribers();
            }
            if(have_to_pub) map_pub.publish(my_map);
        }
    }else creat_fun=1;
}
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
// main
int main(int argc, char** argv) 
{ 
    //
    ros::init(argc, argv, "mvibot_master");   // creat mvibot_node
    ros::NodeHandle nh("~");
    nh.getParam("mvibot_seri", mvibot_seri);
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
                        //cout<<"Data get:\t"<<string_return<<endl;
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
                //cout<<"Data send:\t"<<byte_to_socket<<endl;
                send(sock, byte_to_socket.c_str() ,byte_to_socket.length(), MSG_NOSIGNAL );
            }
	       
    unlock();
}
void process_data(string data){
    if(data[0]=='*'){
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
                break;
            }
        }

    }else{

    }
    //
}
void function1(){
    // don't set delay bigger than timeset in this function,timer set min 0.001 s
        lock();
            printf("Action function 1\n"); 
            time_now("At:");
            pub_map(1);
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
void function3(){
    // don't set  delay bigger than timeset in this function,timer set min 0.001 s
        lock();
            // printf("Action function 3\n"); 
            // time_now("At:");
        unlock();
        // read data from socket
        read_data_socket_server();
}
void function4(){
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
            pub_map(0);
        unlock();
}




