#include"../src/src/common/libary/libary_ros.h"
#include"../src/src/common/libary/libary_basic.h"
#include "../src/src/common/thread_v2/thread_v2.h"
//
void msgf(const std_msgs::String & msg){
    lock();
        cout<<msg.data<<endl;
    unlock();
}
//
void function1();
void function2();
int main(int argc, char** argv){
    //
    ros::init(argc, argv, "mvibot_test");
    //creat thread
    std::thread thread1,thread2,thread3,thread4;
    my_thread my_thread1("Thread 1",1.0,function1,false,-1);
    my_thread my_thread2("Thread 2",0.05,function2,false,-1);
    //start thread
    my_thread1.start(thread1);
    my_thread2.start(thread2);
    //
    ros::NodeHandle n1,n2,n3;
    // sensor check topic
    ros::Subscriber sub1 = n1.subscribe("/msg", 1, msgf);
    ros::spin();
    return 0;
}
void function1(){
   lock();
        cout<<"Function 1 at: "<<get_time()<<endl;
   unlock();
}
void function2(){
    lock();
        cout<<"Function 2 at: "<<get_time()<<endl;      
    unlock();
}