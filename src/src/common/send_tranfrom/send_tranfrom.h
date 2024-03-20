
#include"../libary/libary_ros.h"
#include"../ros_timenow/ros_timenow.h"
using namespace std;

#if !defined(send_tranfrom_define)
    void send_tranfrom(float x, float y, float z, float w, string  name, string name2){

      static tf2_ros::TransformBroadcaster br;
      static geometry_msgs::TransformStamped transformStamped;
      //transformStamped.header.stamp = ros_timenow();
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = name;
      transformStamped.child_frame_id = name2;

      transformStamped.transform.translation.x = x;
      transformStamped.transform.translation.y = y;
      transformStamped.transform.translation.z = 0;

      transformStamped.transform.rotation.x = 0;
      transformStamped.transform.rotation.y = 0;
      transformStamped.transform.rotation.z = z;
      transformStamped.transform.rotation.w = w;
      br.sendTransform(transformStamped);

    }
    void send_tranfrom2(double x, double y, double z, double w, string  name, string name2){

      static tf2_ros::TransformBroadcaster br;
      static geometry_msgs::TransformStamped transformStamped;
      //
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = name;
      transformStamped.child_frame_id = name2;

      transformStamped.transform.translation.x = x;
      transformStamped.transform.translation.y = y;
      transformStamped.transform.translation.z = 0;

      transformStamped.transform.rotation.x = 0;
      transformStamped.transform.rotation.y = 0;
      transformStamped.transform.rotation.z = z;
      transformStamped.transform.rotation.w = w;
      br.sendTransform(transformStamped);

    }
    #define send_tranfrom_define 1
#endif