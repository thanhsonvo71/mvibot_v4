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
#define PORT 9083
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
        //
    if(n_client>0){
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
    }    
}
int main(int argc, char** argv) 
{ 
    //
    ros::init(argc, argv, "mvibot_transport_mapping");   // creat mvibot_node
    ros::NodeHandle nh("~");
    nh.getParam("mvibot_seri", mvibot_seri);
    //
    modify_socket();
    //
    int res;
    res=pthread_create(&p_process1,NULL,process1,NULL);
    res=pthread_create(&p_process2,NULL,process2,NULL);
    res=pthread_create(&p_process3,NULL,process3,NULL);
    res=pthread_create(&p_process4,NULL,process4,NULL);
    //
    ros::NodeHandle n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17,n18,n19,n20,n21;
    ros::Subscriber sub1 = n1.subscribe("/"+mvibot_seri+"/map", 1, mapf);
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
                //cout<<"Data send:\t"<<byte_to_socket<<endl;
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

    }else{
       
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
}
void function6(){
}