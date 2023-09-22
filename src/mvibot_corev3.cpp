#define mvibot_core_define 1
// must have 
#include "src/common/libary/libary_basic.h"
#include "src/common/libary/libary_ros.h"
#include "src/common/string_Iv2/string_Iv2.h"
#include "src/common/thread_v2/thread_v2.h"
// lib function
#include "src/mvibot_core/led/led.h"
#include "src/mvibot_core/battery/battery.h"
#include "src/mvibot_core/music/music.h"
#include "src/mvibot_core/gpio/gpio.h"
#include "src/mvibot_core/odom/odom.h"
#include "src/mvibot_core/config/config.h"
#include "src/mvibot_core/sensor/sensor.h"
#include "src/mvibot_core/motor/motor.h"
#include "src/mvibot_core/speed_control/speed_control.h"
#include "src/mvibot_core/hook/hook.h"
#include "src/mvibot_core/process_data_uart/process_data_uart.h"
#include "src/mvibot_core/mvibot_core_init.h"
//
using namespace std;
// socket var
#define PORT 9080 
struct sockaddr_in address; 
int sock = 0; 
struct sockaddr_in serv_addr; 
char add[225]="127.0.0.1";
float time_out_socket=0;
int data_socket_error=0;
int data_socket_ready=0;
void read_data_socket();
void send_data_socket(std::vector<uint8_t> data_tranf);
void process_data_socket();
int modify_socket();
// create function system
void function1();
void function2();
void function3();
void function4();
void time_now(string data);
// IAM & rqt_test
void pub_IAM(string name){
    static ros::NodeHandle n,n1;
    static ros::Publisher iam_pub = n.advertise<std_msgs::String>("/IAM",100);
    static ros::Publisher robot_status_string = n1.advertise<std_msgs::String>("/robot_status_string",100);
    static std_msgs::String iam_msg;
    static float creat_fun=0;
        if(creat_fun==1){
                //
                iam_msg.data=name+"|robot";
                iam_pub.publish(iam_msg);
                //
        }else {
            creat_fun=1;
        }
}
void pub_rqt_data(){
    static ros::NodeHandle n;
    static ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("/"+mvibot_seri+"/data_rqt",1);
    static float creat_fun=0;
    static std_msgs::Float32MultiArray data;
        if(creat_fun==1){
                // // data.data.resize(19);
                // // data.data[0]=ivr;
                // // data.data[1]=ivl;
                // // data.data[2]=vl_out;
                // // data.data[3]=vr_out;
                // //if(data.data.size()>=4)
                // rqt_data.data.resize(10);
                // rqt_data.data[4]=time_out_socket;
                // rqt_data.data[1]=1;
                // pub.publish(rqt_data);
        }else creat_fun=1;
}
// receive msg
void scan1_check(const sensor_msgs::LaserScan& msg){
    lock();
      radar1_live_status=1;
    unlock();   
}
void scan2_check(const sensor_msgs::LaserScan& msg){
    lock();
      radar2_live_status=1;
    unlock();
}
void camera1_check(const sensor_msgs::LaserScan& msg){
    lock();
      camera1_live_status=1;
    unlock();
}
void camera2_check(const sensor_msgs::LaserScan& msg){
    lock();
      camera2_live_status=1;
    unlock();
}
void imu_listen(const sensor_msgs::Imu & msg){
    lock(); 
        imu_msg=msg;
    unlock();
}
void set_v_w_cmd_vel(const geometry_msgs::Twist& msg){
    lock();
        time_out_cmd_vel=0;
        if(mvibot_sensor_ready==1){
            v_set1=msg.linear.x;
            w_set1=msg.angular.z;
            if(fabs(v_set1)>=v_max) v_set1=fabs(v_set1)/v_set1*v_max;
            if(fabs(w_set1)>=w_max) w_set1=fabs(w_set1)/w_set1*w_max;
        }
    unlock();
}
void set_led(const std_msgs::Float32MultiArray & msg){
    lock();
        red=msg.data[0]/255*100;
        green=msg.data[1]/255*100;
        blue=msg.data[2]/255*100;
        //
        led_l=msg.data[3];
        led_r=msg.data[4];
        led_b=msg.data[5];
        led_f=msg.data[6];
    unlock();
}
void music_namef(const std_msgs::String& msg){
    lock();
     static string cmd;
     cmd="";
     cmd="wget -O "+define_path+"mp3/custom.mp3 "+msg.data+" &";
     system(cmd.c_str());
    unlock();
}
void start_music(const std_msgs::Float32& msg){
    lock();
        if(msg.data==0) start_music_n=0;
        if(msg.data==1)	start_music_n=1;
        if(msg.data==2) start_music_n=2;
        if(msg.data==3) start_music_n=3;
        if(msg.data==4) start_music_n=4;
    unlock();
}
void motor_enablef(const std_msgs::String& msg){
    lock();
        if(msg.data=="0") motor_enable=0;
        if(msg.data=="1") motor_enable=1;
    unlock();
}
void motor_breakf(const std_msgs::String& msg){
    lock();
        if(msg.data=="0") motor_break=0;
        if(msg.data=="1") motor_break=1;
    unlock();
}
void motor_resetf(const std_msgs::String& msg){
    lock();
        if(msg.data=="1") motor_reset=1;
    unlock();
}
void output_user_setf(const std_msgs::String& msg){
    lock();
            cout<<msg.data<<endl;
            static int k=0;
            static string name;
            static string data;
            for(int i=0;i<msg.data.length();i++){
                if(msg.data[i]=='('){
                    name="";
                    data="";
                    k=0;
                    for(int j=i+1;j<msg.data.length();j++){
                        if(msg.data[j]==')') {
                            i=j;
                            break;
                        }
                        else {
                            if(msg.data[j]!='|'){
                                if(k==0) name=name+msg.data[j];
                                else data=data+msg.data[j];
                            }else k++;
                        }
                        i=j;
                    }
                }
                try{
                    static int num;
                    static int value;
                    num=stoi(name,0,10);
                    value=stoi(data,0,10);
                    output_user.data[num]=value;
                }catch(const std::exception& e){
                    std::cerr << e.what() << '\n';
                }
            }
    unlock();
}
void output_user_set_stringf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","||","");
        if(data.data1.size()>=2){
            if(data.data1[0]==mvibot_seri){
                static std_msgs::String send;
                send.data=data.data1[1];
                output_user_setf(send);
            }
        }
    unlock();
}
void robot_shutdownf(const std_msgs::String& msg){
    lock();
        if(msg.data[0]=='1') robot_shutdown=1;
        if(msg.data[0]=='2') robot_shutdown=2;
    unlock();
}
void battery_big_chargef(const std_msgs::String& msg){
    lock();
        if(msg.data[0]=='1') battery1_charge=1;
        else battery1_charge=0;
    unlock();
}
void battery_small_chargef(const std_msgs::String& msg){
    lock();
        if(msg.data[0]=='1') battery2_charge=1;
        else battery2_charge=0;
    unlock();
}
void robot_emgf(const std_msgs::String & msg){
    lock();
         if(msg.data[0]=='1') robot_emg=1;
    unlock();
}
void robot_load_configf(const std_msgs::String & msg){
    lock();
        if(msg.data=="1"){
            robot_load_config();
        }
    unlock();
}
void robot_update_softwaref(const std_msgs::String & msg){
    lock();
         //if(msg.data=="1") software_update=1;
    unlock();
}
int main(int argc, char** argv){
    // update time to memorry
    ts_scan_sensor=1.0;
    ts_pid=0.05;
    ts_speed_control=0.05;
    //
    ros::init(argc, argv, "mvibot_corev3");   // creat mvibot_node
    ros::NodeHandle nh("~");
    nh.getParam("mvibot_seri", mvibot_seri);
    nh.getParam("mode", mode);
    while(modify_socket()==-1){
        sleep(1);
    }
    sleep(2);
    //
    input_user.data.resize(num_in_put_user+3);
    output_user.data.resize(num_out_put_user+5);
    //
    data_tran.resize(num_byte_pc_stm);
    data_tran[0]=byte_start0;
    data_tran[1]=byte_start1;
    data_tran[2]=byte_start2;
    data_tran[data_tran.size()-3]=byte_end0;
    data_tran[data_tran.size()-2]=byte_end1;
    data_tran[data_tran.size()-1]=byte_end2;
    // setup odom msg
    odom_wheel.header.frame_id =mvibot_seri+"/odom";
    odom_wheel.child_frame_id=mvibot_seri+"/base_footprint";
    odom_wheel.pose.pose.position.z = 0.0;
    imu_msg.header.frame_id =mvibot_seri+"_camera2_imu_optical_frame";
    imu_msg.linear_acceleration.x=-9.8;
    // ser cov
    for(int i=0;i<36;i++){
        if(i==0|i==7|i==35){
            odom_wheel.pose.covariance[i]=0.01;
            if(i==35)  odom_wheel.pose.covariance[i]=M_PI/60; 
        }
        if(i==0|i==35){
            odom_wheel.twist.covariance[i]=0.02;
        }
    }
    //
    static std_msgs::String update_config;
    update_config.data="1";
    robot_load_configf(update_config);
    //creat thread
    std::thread thread1,thread2,thread3,thread4;
    my_thread my_thread1("Thread 1",1.0,function1,false,-1);
	my_thread my_thread2("Thread 2",0.05,function2,false,-1);
    my_thread my_thread3("Thread 3",0.05,function3,false,-1);
    my_thread1.start(thread1);
    my_thread2.start(thread2);
    my_thread3.start(thread3);
    //msg ros
    ros::NodeHandle n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17,n18,n19,n20,n21,n22,n23;
    // sensor check topic
    ros::Subscriber sub2 = n2.subscribe("/"+mvibot_seri+"/laser/scan1", 1, scan1_check);
    ros::Subscriber sub3 = n3.subscribe("/"+mvibot_seri+"/laser/scan2", 1, scan2_check);
    ros::Subscriber sub4 = n4.subscribe("/"+mvibot_seri+"/camera1/scan", 1, camera1_check);
    ros::Subscriber sub5 = n5.subscribe("/"+mvibot_seri+"/camera2/scan", 1, camera2_check);
    // pose lasser from icp and imu
    ros::Subscriber sub7 = n7.subscribe("/"+mvibot_seri+"/camera2/camera/imu", 1, imu_listen);
    // cmd_vel_robot
    ros::Subscriber sub8 = n8.subscribe("/"+mvibot_seri+"/cmd_vel", 1, set_v_w_cmd_vel);
    // set led robot
    ros::Subscriber sub9 = n9.subscribe("/"+mvibot_seri+"/set_led", 1, set_led);
    // music robot 
    ros::Subscriber sub10 = n10.subscribe("/"+mvibot_seri+"/music_name", 1, music_namef);
    ros::Subscriber sub11 = n11.subscribe("/"+mvibot_seri+"/music_start", 1, start_music);
    // motor robot 
    ros::Subscriber sub12 = n12.subscribe("/"+mvibot_seri+"/motor_enable", 1, motor_enablef);
    ros::Subscriber sub13 = n13.subscribe("/"+mvibot_seri+"/motor_break", 1, motor_breakf);
    ros::Subscriber sub14 = n14.subscribe("/"+mvibot_seri+"/motor_reset", 1, motor_resetf);
    // io robot
    ros::Subscriber sub15 = n15.subscribe("/"+mvibot_seri+"/output_user_set", 1, output_user_setf);
    ros::Subscriber sub21 = n21.subscribe("/output_user_set_string", 1, output_user_set_stringf);
    // shut down robot
    ros::Subscriber sub16 = n16.subscribe("/"+mvibot_seri+"/robot_shutdown", 1, robot_shutdownf);
    // battery charge
    ros::Subscriber sub17 = n17.subscribe("/"+mvibot_seri+"/battery_big_charge", 1, battery_big_chargef);
    ros::Subscriber sub18 = n18.subscribe("/"+mvibot_seri+"/battery_small_charge", 1, battery_small_chargef);
    // robot emg
    ros::Subscriber sub19 = n19.subscribe("/"+mvibot_seri+"/robot_emg",1,robot_emgf);
    // robot update config
    ros::Subscriber sub20 = n20.subscribe("/"+mvibot_seri+"/robot_load_config",1,robot_load_configf);
    ros::Subscriber sub22 = n22.subscribe("/"+mvibot_seri+"/robot_update_software",1,robot_update_softwaref);
    ros::spin();
    //
    thread1.join();
    thread2.join();
    thread3.join();
    close(sock);
    return 0;
}
//
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
void read_data_socket(){
        static int num_byte;
        static uint8_t data_receive_socket_local[999]={};
        static int count;
        static int data_error;
        memset(data_receive_socket_local,0,sizeof data_receive_socket_local);
        num_byte=read( sock , data_receive_socket_local, 999);
        lock();
            if(num_byte>0){
                data_receive.resize(num_byte);
                for(int j=num_byte;j>=2;j--){
                            //
                            count=0;
                            data_error=1;
                            if(data_receive_socket_local[j]==byte_end2 &  data_receive_socket_local[j-1]==byte_end1 & data_receive_socket_local[j-2]==byte_end0){
                                for(int k=j;k>=0;k--){
                                    count++;
                                    if(data_receive_socket_local[k]==byte_start0 & data_receive_socket_local[k+1]==byte_start1 & data_receive_socket_local[k+2]==byte_start2 ){
                                        data_error=0;
                                        data_receive.resize(count);
                                        for(int h=k;h<=j;h++){
                                            data_receive[h-k]=data_receive_socket_local[h];
                                        }
                                        data_socket_ready=1;
                                        break;
                                    }
                                }
                                if(data_error==0) break;
                            }
                }
                //if(data_socket_ready==1) view_data("data from socket: ",data_receive);
                //time_now(to_string(num_byte));
            }
        unlock();
}
void send_data_socket(std::vector<uint8_t> data_tranf){
    lock();
        //uint8_t data_tran_socket[data_tranf.size()];
        static uint8_t data_tran_socket[num_byte_pc_stm];
        memset(data_tran_socket,0,sizeof data_tran_socket);
        for(int i=0;i<data_tranf.size();i++){
            data_tran_socket[i]=data_tranf[i];
        }
        //view_data("send data to socket :",data_tranf);
        send(sock , data_tran_socket , data_tranf.size() , 0 );
    unlock();
}
void process_data_socket(){
    lock();
        // process recive socket
        if(data_socket_ready==1){
               static int uart_check;
               uart_check=0;
               time_out_socket=0;
               for(int i=3;i<num_byte_stm_pc-3;i++){
                    static uint8_t byte_check=byte_uart_time_out;
                    if(data_receive[i]!=byte_check){
                        uart_check=1;
                        break;
                    }
               }
               if(uart_check==1){
                    // process to read socket <- data_recevice
                    // process data pc <- stm
                    uart_live=1;
                    process_data_uart_read();
               }else uart_live=0;
        }else{
            time_out_socket+=(float)0.05;
            if(time_out_socket>=0.250) time_out_socket=0.250;
            if(time_out_socket>=0.250) uart_live=0;
        }
        data_socket_ready=0;
        // process to send socket -> data_tran
            // control cmd to stm
            led_control();
            speed_robot_control();
            pid_motor();
            torque_control();
            // process data pc -> stm
            process_data_uart_write();
        //
        pub_output_user_status();
        pub_input_user_status();
        pub_hook_switch(hook_switch);
        //
    unlock();
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
    lock();
        pub_IAM(mvibot_seri);
        pub_output_user_string_status();
        pub_input_user_string_status();
        pub_motor_right_status();
        pub_motor_left_status();
        pub_battery_status();
        pub_battery_cell_status();
        pub_battery_small_status();
        pub_robot_status();
        pub_robot_config();
        music_control();
        check_sensor();
    unlock();
        action_sensor();
}
void function2(){
    process_data_socket();
    send_data_socket(data_tran);
    lock();
            //cout<<"Function 2  at|"<<get_time()<<endl;
            caculate_and_send_odom();
            send_hook_frame(hook_switch);
    unlock();
}
void function3(){
    lock();
    unlock();
    read_data_socket();
}
void function4(){
    lock();
    unlock();
}