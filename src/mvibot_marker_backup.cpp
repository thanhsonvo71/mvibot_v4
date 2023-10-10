// must have 
#include "src/common/libary/libary_basic.h"
#include "src/common/thread/thread.h"
#include "src/common/string_Iv2/string_Iv2.h"
//
#include "src/mvibot_marker/mvibot_marker_init.h"
#include "src/mvibot_marker/data/data.h"
#include "src/mvibot_marker/marker/marker.h"
#include "src/mvibot_marker/marker/vl.h"
#include "src/mvibot_marker/marker/bar.h"
#include "src/mvibot_marker/marker/l.h"
#include "src/mvibot_marker/robot/robot.h"

using namespace std;
// settime process
long double ts_process1=0.05; //time set for process1
long double ts_process2=0.05; //time set for process2
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
//
void scan1f(const sensor_msgs::LaserScan &msg){
    lock();
        if(start==1 & my_marker.marker_dir=="front_ward"){
            static int n;
            n=my_marker.my_data.my_scan.size();
            if(n<my_marker.my_data.num_msg){
                cout<<"get_laser1_msg:"<<n+1<<endl;
                my_marker.my_data.my_scan.resize(my_marker.my_data.my_scan.size()+1);
                my_marker.my_data.my_scan[n]=msg;
            }
        }
    unlock();
}
void scan2f(const sensor_msgs::LaserScan &msg){
    lock();
        if(start==1){
            static int n;
            n=my_marker.my_data.my_scan.size();
            if(n<my_marker.my_data.num_msg & my_marker.marker_dir=="back_ward"){
                cout<<"get_laser2_msg:"<<n+1<<endl;
                my_marker.my_data.my_scan.resize(my_marker.my_data.my_scan.size()+1);
                // inverse before add
                static sensor_msgs::LaserScan msg2;
                msg2=msg;
                for(int i=0;i<msg2.ranges.size()/2;i++){
                    static float a;
                    a=msg2.ranges[i];
                    msg2.ranges[i]=msg2.ranges[i+msg2.ranges.size()/2];
                    msg2.ranges[i+msg2.ranges.size()/2]=a;
                }
                my_marker.my_data.my_scan[n]=msg2;
            }
        }
    unlock();
}
void scan(const sensor_msgs::LaserScan &msg){
    lock();
        if(start==1){
            scan_safe=msg;
        }
    unlock();
}
void start_markerf(const std_msgs::String& msg){
    lock();
        if(msg.data=="0"){
            robot_emg();
            pub_cmd_vel(0,0);
            start=0;
        }else if (msg.data=="1"){
            if(start==0){
                start=1;
                my_marker.reset(1);
            } 
        }else{
            if(start==0){
                robot_emg();
                pub_cmd_vel(0,0);
                start=1;
                my_marker.reset(0);
                process_data(msg.data);
                cout<<msg.data<<endl;
            }
        }
    unlock();
}
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
int main(int argc, char** argv){
    ros::init(argc, argv, "mvibot_marker");
    ros::NodeHandle nh("~");
    nh.getParam("mvibot_seri", mvibot_seri);
    //
    while(my_marker.tranfrom_pose_marker(0)==-1){
        my_marker.send_tranfrom_marker();
        usleep(1e5);
    }
    //
    cout<<"Setup_Finish"<<endl;
    pub_cmd_vel(0,0);
    robot_emg();
    pub_status_marker();
    //
    static int res;
    res=pthread_create(&p_process1,NULL,process1,NULL);
    res=pthread_create(&p_process2,NULL,process2,NULL);

    ros::NodeHandle n1,n2,n3,n4,n5;
    //ros::Subscriber sub1 = n1.subscribe("/"+mvibot_seri+"/msg", 1, msgf);    
    ros::Subscriber sub2 = n2.subscribe("/"+mvibot_seri+"/laser/base_link/scan1", 1, scan1f); 
    ros::Subscriber sub3 = n3.subscribe("/"+mvibot_seri+"/laser/base_link/scan2", 1, scan2f);
    ros::Subscriber sub4 = n4.subscribe("/"+mvibot_seri+"/start_marker", 1, start_markerf); 
    ros::Subscriber sub5 = n5.subscribe("/"+mvibot_seri+"/laser/scan", 1, scan); 
    ros::spin(); 
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
        robot_position=get_position(mvibot_seri+"/odom",mvibot_seri+"/base_footprint");
        if(start==1){
            my_marker.action();
        }
        pub_status_marker();
        pub_status();
        //cout<<"Timer function!"<<endl;
    unlock();

}
void function2(){
    lock();
       //cout<<"Timer2 function!"<<endl;
    unlock();
}
void function3(){
       
}
void function4(){
      
}
void function5(){

}