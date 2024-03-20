#include "src/libary/libary_basic.h"
#include "src/libary/libary_ros.h"
#include "src/thread/thread.h"
//
using namespace  std;
string mvibot_seri;
string system_command;
// socket var 
#define PORT 9080 
int socket_port[1000];
int server_fd;
struct sockaddr_in address; 
int opt = 1; 
int addrlen = sizeof(address); 
int n_client;
char str_cli_ip[INET_ADDRSTRLEN];
struct sockaddr_in* ip_client;
class port_receive{
    public:
        string ip;
        int port;
};
port_receive port_clinet_receive[1000];
float time_out_socket=0.0;
std::vector<uint8_t>  data_tran_socket={};
std::vector<uint8_t>  data_tran_socket_defaunt={};
std::vector<uint8_t>  data_receive_socket={};
int max_n_clinet=1000;
// thread var
static pthread_mutex_t process_mutex;	
static pthread_t p_process1;
static pthread_t p_process2;
static pthread_t p_process3;
static pthread_t p_process4;
static long double ts1=0.05;
static long double ts2=0.05;
static long double ts3=0.05;
static long double ts4=0.05;
//
string time_now(string name);
void lock();
void unlock();
void modify_socket();
void send_data_socket_server();
void read_data_socket_server();
void function2();
void function3();
void function4();
//
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
                system_command="echo ";
                system_command=system_command+"'"+time_now("Accept socket connection at:")+"' >> /home/mvibot/catkin_ws/src/mvibot/history/startup.log";
                system(system_command.c_str());
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
int main(int argc, char** argv) 
{ 
    //
    modify_socket();
    //
    int res;
    res=pthread_create(&p_process1,NULL,process1,NULL);
    res=pthread_create(&p_process2,NULL,process2,NULL);
    res=pthread_create(&p_process3,NULL,process3,NULL);
    res=pthread_create(&p_process4,NULL,process4,NULL);
    while (1)
    {
        sleep(1);
    }
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
        goto A;
        exit(EXIT_FAILURE); 
    }
}
void send_data_socket_server(){
    lock();
        if(n_client>0){
            // static uint8_t data_send_socket_local[num_byte_stm_pc];
            // std::copy(data_tran_socket.begin(), data_tran_socket.end(), data_send_socket_local);
            // cout<<"data send socket:";
            // for(int i=0;i<sizeof data_send_socket_local;i++)
            // printf("0x%x ",data_send_socket_local[i]);
            // cout<<endl;
            // send(socket_port[n_client-1], data_send_socket_local,sizeof data_send_socket_local, MSG_NOSIGNAL );  //MSG_NOSIGNAL no exit when the core close socket
            // char * test="Helllo";
	    send(socket_port[n_client-1], test ,sizeof test, MSG_NOSIGNAL );
	}
    unlock();
}
void read_data_socket_server(){
        if(n_client>0){
                static int num_byte;
                static int count;
                static int data_error;
                static uint8_t support[999];
                static int i;
                memset(support, 0, sizeof support); 
                i=n_client-1;
                if(n_client>0) num_byte=read( socket_port[i], support, 999);    
                lock();     
                    if(num_byte>0){
        
                       
                    }
                unlock();
        }
}
void function2(){
	// don't set delay big than time_set in this function,timer set min 0.001 s
        lock();
            printf("Action function 2\n"); 
            time_now("At:");
        unlock();
        // communicate with stm
}
void function3(){
    // don't set delay big than in this function,timer set min 0.001 s
        lock();
            printf("Action function 3\n"); 
            time_now("At:");
        unlock();
        // read data socket
        read_data_socket_server();
}
void function4(){
    // don't set delay big than in this function,timer set min 0.001 s
        lock();
            printf("Action function 4\n"); 
            time_now("At:");
        unlock();
        // send data socket
        send_data_socket_server();
}
