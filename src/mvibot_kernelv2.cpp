// must have 
#include "src/common/libary/libary_basic.h"
#include "src/common/thread_v2/thread_v2.h"
#include "src/common/string_Iv2/string_Iv2.h"
//
#include "src/mvibot_kernel/uart/uart.h"
#include "src/mvibot_kernel/backup/backup.h"
#include "src/mvibot_kernel/keyboard/keyboard.h"
#include "src/mvibot_kernel/monitor/monitor.h"
#include "src/mvibot_kernel/security/security.h"
//
#include "src/mvibot_kernel/mvibot_kernel_init.h"
//  
using namespace std;
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
int max_n_clinet=1000;
//
// create function system
void modify_socket();
void send_data_socket_server();
void check_time_out_socket();
void read_data_socket_server();
void function1();
void function2();
void function3();
void function4();
void function5();
void time_now(string data);
//
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
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, (const char*)&tv, sizeof tv))
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
        if(n_client>0){
            static uint8_t data_send_socket_local[num_byte_stm_pc];
            std::copy(data_tran_socket.begin(), data_tran_socket.end(), data_send_socket_local);
            cout<<"data send socket:";
            for(int i=0;i<sizeof data_send_socket_local;i++)
            printf("0x%x ",data_send_socket_local[i]);
            cout<<endl;
            send(socket_port[n_client-1], data_send_socket_local,sizeof data_send_socket_local, MSG_NOSIGNAL );  
            //MSG_NOSIGNAL no exit when the core close socket
        }
}
void check_time_out_socket(){
        if(n_client>0){            
            time_out_socket+=(float)ts_socket;
            if(time_out_socket>=0.250){
                time_out_socket=0.250;
                cout<<"Time out socket\n";
                data_tran_uart=data_tran_uart_defaunt;
            }else{
                data_tran_uart=data_receive_socket;
            }
        }else{
            data_tran_uart=data_tran_uart_defaunt;
        }
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
                        time_out_socket=0;
                        //
                        cout<<"data receive socket: ";
                        for(int j=0;j<num_byte;j++){
                            printf("0x%x ",support[j]);
                        }
                        cout<<endl;
                        //
                        for(int j=num_byte;j>=2;j--){
                            count=0;
                            data_error=1;
                            if(support[j]==byte_end2 &  support[j-1]==byte_end1 &  support[j-2]==byte_end0){
                                for(int k=j;k>=0;k--){
                                    count++;
                                    if(support[k]==byte_start0 & support[k+1]==byte_start1 & support[k+2]==byte_start2){
                                        data_error=0;
                                        data_receive_socket.resize(count);
                                        for(int h=k;h<=j;h++){
                                            data_receive_socket[h-k]=support[h];
                                        }
                                        break;
                                    }
                                }
                                if(data_error==0) break;
                            }
                        }
                        //  
                        if(data_error==1) data_receive_socket=data_tran_uart_defaunt;
                        else{
                                cout<<"data process socket: ";
                                for(int i=0;i<data_receive_socket.size();i++)
                                printf("0x%x ",data_receive_socket[i]);
                                cout<<endl;
                        }
                    }
                unlock();
        }
}
int main(int argc, char** argv){
    // check match hardware
    //if(check_security()==0){
    //    cout<<"Don 't match hardware!"<<endl;
    //    security_code();
    //}
    get_list_monitor();
    // history startup
    system_command="echo ";
    system_command=system_command+"robot startup at: `date` "+">> "+define_path+"history/startup.log";
    system(system_command.c_str());
    // send mail other
    system_command="scp /home/mvibot/catkin_ws/src/mvibot/history/battery.log /home/mvibot/catkin_ws/src/mvibot/history/battery_send.log";
    system(system_command.c_str());
    //
    system_command="cd /home/mvibot/catkin_ws/src/mvibot/history/ &&";
    system_command+=" (echo $(date) | mail -s \"Battery history\" mvpkouki@gmail.com -A battery_send.log )";
    system(system_command.c_str());
    // modify uart
    serialPort.SetTimeout(5);
    uart_open();
    // update time scan
    modify_socket();
    ts_socket=0.05;
    ts_uart=0.05;
    //modify data
    data_tran_uart_defaunt.resize(num_byte_pc_stm);
    data_tran_uart_defaunt[0]=byte_start0;
    data_tran_uart_defaunt[1]=byte_start1;
    data_tran_uart_defaunt[2]=byte_start2;
    data_tran_uart_defaunt[data_tran_uart_defaunt.size()-3]=byte_end0;
    data_tran_uart_defaunt[data_tran_uart_defaunt.size()-2]=byte_end1;
    data_tran_uart_defaunt[data_tran_uart_defaunt.size()-1]=byte_end2;
    //
    data_tran_uart_defaunt[robot_color_red]=100;
    data_tran_uart_defaunt[robot_color_green]=100;
    data_tran_uart_defaunt[robot_color_blue]=100;
    data_tran_uart_defaunt[robot_led_left]=1;
    data_tran_uart_defaunt[robot_led_right]=1;
    data_tran_uart_defaunt[robot_led_back]=1;
    //data_tran_uart_defaunt[41]=1;
    //
    data_tran_socket_defaunt.resize(num_byte_stm_pc);
    data_tran_socket_defaunt[0]=byte_start0;
    data_tran_socket_defaunt[1]=byte_start1;
    data_tran_socket_defaunt[2]=byte_start2;
    data_tran_socket_defaunt[data_tran_socket_defaunt.size()-3]=byte_end0;
    data_tran_socket_defaunt[data_tran_socket_defaunt.size()-2]=byte_end1;
    data_tran_socket_defaunt[data_tran_socket_defaunt.size()-1]=byte_end2;
    for(int i=0;i<data_tran_socket_defaunt.size();i++){
        static uint8_t data;
        data=data_tran_socket_defaunt[i];
        if(data!=byte_start0 & data != byte_start1 & data!=byte_start2){
            if(data!=byte_end0 & data != byte_end1 & data!=byte_end2){
                data_tran_socket_defaunt[i]=byte_uart_time_out;
            }
        }
    }
    // creat thread
    thread thread1,thread2,thread3,thread4,thread5;
    my_thread my_thread1("Thread1",0.05,function1,false,-1);
    my_thread my_thread2("Thread2",0.05,function2,false,-1);
    my_thread my_thread3("Thread3",0.05,function3,false,-1);
    my_thread my_thread4("Thread4",0.05,function4,false,-1);
    my_thread my_thread5("Thread5",0.05,function5,false,-1);
    my_thread1.start(thread1);
    my_thread2.start(thread2);
    my_thread3.start(thread3);
    my_thread4.start(thread4);
    my_thread5.start(thread5);
    while (1)
    {
        sleep(1);
    }
    return 0;
}
//
void time_now(string name){
    lock();
        static struct timespec realtime;
        clock_gettime(CLOCK_REALTIME, &realtime);
        cout<<name+"|Time:"<<std::fixed << std::setprecision(5)<<((long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9)<<endl;
    unlock();
}
void function1(){
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
            lock();
                ip_client= (struct sockaddr_in*)&address;
                inet_ntop(AF_INET, &ip_client->sin_addr, str_cli_ip, INET_ADDRSTRLEN);
                //
                port_clinet_receive[n_client].ip=str_cli_ip;
                port_clinet_receive[n_client].port=ntohs(ip_client->sin_port);
                n_client++;
                cout<<"accept: "<<n_client<<endl;
                //
                system_command="echo ";
                system_command=system_command+"robot accept socket at: `date` "+">> "+define_path+"history/startup.log";
                system(system_command.c_str());
            unlock();
        }
        usleep(1000000);
    }  

}
void function2(){
        lock();
           cout<<"Function 2 at |"<<get_time()<<endl;
        unlock();
        // communicate with stm
        lock();
            uart_write();
        unlock();
            usleep(30000);
        // check time out data from stm
        lock();
            uart_read();
            uart_timeout();
        unlock();
}
void function3(){
        lock();
           cout<<"Function 3 at |"<<get_time()<<endl;
        unlock();
        // read data socket
        read_data_socket_server();
}
void function4(){
        lock();
            cout<<"Function 4 at |"<<get_time()<<endl;
            // check time out socket
            check_time_out_socket();
            // send data socket
            send_data_socket_server();
        unlock();
}
void function5(){
    lock();
        cout<<"Function 5 at |"<<get_time()<<endl;
        // bacup request
        if(request_backup==1){
            static int finish_backup=0;
            if(finish_backup==0){
                backup();
                finish_backup=1;
            }else{
                time_out_backup-=0.05;
                if(time_out_backup<=0){
                    robot_backup_reboot=1;
                }
            }
        }
        //
        //if(check_monitor_connect()){
        //    cout<<"Some one is connect to output display..."<<endl;
        //    send_mail("mvpkouki@gmail.com",name_seri_fix+" Alarm Security",name_seri_fix+" : someone is plugined display port At `date`","");
        //    security_code();
        //}
    unlock();
}
