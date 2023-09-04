#include"../libary/libary_ros.h"
using namespace std;
float *get_position(string name1, string name2){
        static float data[4];
        //get position
        static tf::StampedTransform transform;
        static ros::Time t;
        static tf::TransformListener listen;
        static float x,y,z,thz,thw;
        static string data_position;
        try
        {
                try {
                    listen.waitForTransform(name1, name2, ros::Time(0), ros::Duration(0.1));
                    listen.lookupTransform (name1, name2, ros::Time(0), transform);
                    x=transform.getOrigin().x();
                    y=transform.getOrigin().y();
                    z=transform.getOrigin().z();
                    thz=transform.getRotation().getZ();
                    thw=transform.getRotation().getW();
	        } catch(ros::Exception &e){
                        x=-1; y=-1; thz=-1; thw=-1;
                        ROS_ERROR("Error occured: %s ", e.what());
                }
        }
        catch(const std::exception& e)
        {
                std::cerr << e.what() << '\n';
                x=-1; y=-1; thz=-1; thw=-1;
        }
        data[0]=x; data[1]=y; data[2]=thz; data[3]=thw;
        return data;
}
float *get_position2(string name1, string name2){
        static float data[4];
        //get position
        static tf::StampedTransform transform;
        static ros::Time t;
        static tf::TransformListener listen;
        static float x,y,z,thz,thw;
        static string data_position;
        geometry_msgs::TransformStamped transformStamped;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        try
        {
                try{
                        transformStamped = tfBuffer.lookupTransform(name1,name2,ros::Time(0),ros::Duration(0.0));
                        x=transformStamped.transform.translation.x;
                        y=transformStamped.transform.translation.y;
                        z=transformStamped.transform.translation.z;
                        thz=transformStamped.transform.rotation.z;
                        thw=transformStamped.transform.rotation.w;
                }
                catch (tf2::TransformException &e) {
                        x=-1; y=-1; thz=-1; thw=-1;
                        ROS_ERROR("Error occured: %s ", e.what());
                }

        }
        catch(const std::exception& e)
        {
                std::cerr << e.what() << '\n';
                x=-1; y=-1; thz=-1; thw=-1;
        }
        data[0]=x; data[1]=y; data[2]=thz; data[3]=thw;
        return data;
}
