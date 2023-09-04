#include"../libary/libary_ros.h"
#include"../libary/libary_basic.h"
using namespace std;
#if !defined(ros_timenow_define)
    ros::Time ros_timenow(){
        static ros::Time t;
        static struct timespec realtime;
        clock_gettime(CLOCK_REALTIME, &realtime);
        t.sec=realtime.tv_sec;
        t.nsec=realtime.tv_nsec;
        return t;
    }
    #define ros_timenow_define 1
#endif