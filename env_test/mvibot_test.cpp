#include"../src/src/common/libary/libary_ros.h"
#include"../src/src/common/libary/libary_basic.h"
#include "../src/src/common/thread_v2/thread_v2.h"
#include "../src/src/common/send_tranfrom/send_tranfrom.h"
#include "../src/src/common/get_position/get_position.h"
#include "../src/src/common/string_Iv2/string_Iv2.h"
//
double x=0,y=0,z=0,w=1;
double x_get,y_get,z_get,w_get;
double *data;
int new_update=0;
//
ros::Time t_get,t_send;
void msgf(const std_msgs::String & msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        x=stod(data.data1[0]);
        y=stod(data.data1[1]);
        z=stod(data.data1[2]);
        w=stod(data.data1[3]);
        new_update=1;
    unlock();
}
//
double approximately(double value, int num){
    double value_return;
    int64_t n=pow(10,num);
    int64_t m=round((value*n));
    value_return=((double)m)/n;
    return value_return;
}
int compare_pose(double x1, double y1, double z1, double w1, double x2, double y2, double z2, double w2){
    geometry_msgs::Pose pose_1,pose_2;
    //
    static double thresold_position=0.001;
    static double thresold_angle=0.001;
    if(sqrt(pow(x2-x1,2)+pow(y2-y1,2))<=thresold_position){
        if(sqrt(pow(z2-z1,2)+pow(w2-w1,2))<=thresold_angle){
            return 1;
        }   
    }
    return 0;
}
//
void function1();
void function2();
int main(int argc, char** argv){
    //
    ros::init(argc, argv, "mvibot_test");
    //creat thread
    std::thread thread1,thread2,thread3,thread4;
    my_thread my_thread1("Thread 1",0.05,function1,false,-1);
    my_thread my_thread2("Thread 2",0.05,function2,false,-1);
    //start thread
    my_thread1.start(thread1);
    //my_thread2.start(thread2);
    //
    ros::NodeHandle n1,n2,n3;
    // sensor check topic
    ros::Subscriber sub1 = n1.subscribe("/msg", 1, msgf);
    ros::spin();
    return 0;
}
void function1(){
   lock();
        //cout<<"Function 1 at: "<<get_time()<<endl;
        if(new_update){
            t_send=ros::Time::now();
            new_update=0;
        }
        send_tranfrom(x,y,z,w,"/map","MB22_916b/odom");
        send_tranfrom(x,y,z,w,"MB22_916b/odom","MB22_916b/base_footprint");
   unlock();
}
void function2(){
    lock();
        //cout<<"Function 2 at: "<<get_time()<<endl;      
        data=get_position2("map","MB22_916b/odom");
        std::cout << std::fixed << std::setprecision(9);
        x_get=data[0]; y_get=data[1]; z_get=data[2]; w_get=data[3]; t_get.sec=(uint32_t)data[4]; t_get.nsec=(uint32_t)data[5];
        if(t_get<t_send) cout<<"Error time"<<endl;
        else{
            cout<<x<<"|"<<y<<"|"<<z<<"|"<<w<<endl;
            cout<<x_get<<"|"<<y_get<<"|"<<z_get<<"|"<<w_get<<endl;
        }
        // cout<<t_send.sec<<"|"<<t_send.nsec<<endl;
        // cout<<t_get.sec<<"|"<<t_get.nsec<<endl;
        if(compare_pose(x,y,z,w,x_get,y_get,z_get,w_get)) cout<<"Match"<<endl;
        else cout<<"Don't match"<<endl;
    unlock();
}