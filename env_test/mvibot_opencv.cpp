#include"../src/src/common/libary/libary_ros.h"
#include"../src/src/common/libary/libary_basic.h"
#include "../src/src/common/thread_v2/thread_v2.h"
#include <opencv2/opencv.hpp> 
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "cv_bridge/cv_bridge.h"
#include "zbar.h"
using namespace std;
using namespace cv;
// convert sensor image msg to image format in opencv
Mat image,depth_image;
sensor_msgs::CameraInfo depth_infor;
sensor_msgs::Image depth_msg;
int get_image=0;
int get_depth_image=0;
double get_depth_value(sensor_msgs::Image depth,int x, int y);
//
cv::Point center;
void image_callback(const sensor_msgs::ImageConstPtr &msg){
    lock();
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        //
        image=cv_ptr->image;
        get_image=1;
    unlock();
}
void depth_callback(const sensor_msgs::Image &msg){
    lock();
        //
        depth_msg=msg;
        get_depth_image=1;
        cv_bridge::CvImageConstPtr cv_ptr;
        depth_image=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1).get()->image;
        depth_image.convertTo(depth_image,CV_8U);
    unlock();
}
void depth_infor_callback(const sensor_msgs::CameraInfo & msg){
    lock();
        depth_infor=msg;
    unlock();
}
double get_depth_value(sensor_msgs::Image depth,int x, int y){
    double value_return;
    value_return=(double)(((uint16_t)depth.data[y*depth.width*2+x*2+1]<<8)|(uint16_t)depth.data[y*depth.width*2+x*2])/1000;
    return value_return;
}
void send_tranfrom2(float x, float y, float z, string  name, string name2){
    static tf2_ros::TransformBroadcaster br;
    static geometry_msgs::TransformStamped transformStamped;
    //transformStamped.header.stamp = ros_timenow();
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = name;
    transformStamped.child_frame_id = name2;

    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;

    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
    br.sendTransform(transformStamped);

}
void detect_qr(){
    Mat grayImage;
    cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    //
    // Khởi tạo đối tượng ZBar để quét mã
    zbar::ImageScanner scanner;
    // Tạo một hình ảnh ZBar từ ảnh grayscale
    zbar::Image zbarImage(grayImage.cols, grayImage.rows, "Y800", grayImage.data, grayImage.cols * grayImage.rows);
    // Quét mã trong ảnh
    int n = scanner.scan(zbarImage);
    // Hiển thị thông tin từ mã QR nếu có
    for (zbar::Image::SymbolIterator symbol = zbarImage.symbol_begin(); symbol != zbarImage.symbol_end(); ++symbol) {
        cout << "Type: " << symbol->get_type_name() << "\n";
        cout << "Data: " << symbol->get_data() << "\n";
        // Lấy tọa độ của mã QR code
        std::vector<cv::Point> points;
        for (int i = 0; i < symbol->get_location_size(); ++i) {
            points.emplace_back(symbol->get_location_x(i), symbol->get_location_y(i));
        }
        // Vẽ đường viền xung quanh mã QR code
        cv::polylines(image, points, true, cv::Scalar(0, 255, 0), 2);
        cv::polylines(depth_image, points, true, cv::Scalar(0, 255, 0), 2);
        static float sx,sy;
        sx=0; sy=0;
        for(int i=0;i<points.size();i++){
            cv::circle(image, points[i], 5, cv::Scalar(255, 0 , 0), -1);
            cv::circle(depth_image, points[i], 5, cv::Scalar(255, 0 , 0), -1);
            sx+=(float)points[i].x;
            sy+=(float)points[i].y;
        }
        //cout<<"end"<<endl;
        center.x=(uint16_t)(sx/4);
        center.y=(uint16_t)(sy/4);
        cv::circle(image, center, 5, cv::Scalar(0, 0 ,255), -1);
        cv::circle(depth_image, center, 5, cv::Scalar(0, 0 ,255), -1);
        //
        if(get_depth_image){
            //calculator X,Y,Z
            cv::Matx33d K(depth_infor.K[0], depth_infor.K[1], depth_infor.K[2],
                    depth_infor.K[3], depth_infor.K[4], depth_infor.K[5],
                    depth_infor.K[6], depth_infor.K[7], depth_infor.K[8]);
            //
            double depth_value;
            depth_value=get_depth_value(depth_msg,center.x,center.y);
            cv::Matx31d point_2d(center.x, center.y, 1.0);
            cv::Matx31d point_3d = (K.inv() * point_2d)*depth_value;
            ROS_INFO("QR code (X, Y, Z): (%f, %f, %f)", point_3d(0), point_3d(1), point_3d(2));
            send_tranfrom2(point_3d(0),point_3d(1),point_3d(2),depth_msg.header.frame_id,"/qr_code");
        }
    }
    get_depth_image=0;
    //
    imshow("ROS Depth",depth_image);
    imshow("ROS Image",image);  
}
//
void function1();
void function2();
int main(int argc, char** argv){
    //
    ros::init(argc, argv, "mvibot_opencv");
    //creat thread
    std::thread thread1,thread2,thread3,thread4;
    my_thread my_thread1("Thread 1",1.0,function1,false,-1);
    my_thread my_thread2("Thread 2",0.05,function2,false,-1);
    //start thread
    my_thread1.start(thread1);
    my_thread2.start(thread2);
    //
    namedWindow("ROS Image", cv::WINDOW_NORMAL);
    namedWindow("ROS Depth", cv::WINDOW_NORMAL);
    //
    ros::NodeHandle n1,n2,n3;
    // sensor check topic
    ros::Subscriber sub1 = n1.subscribe("/camera/camera/color/image_raw", 1, image_callback);
    ros::Subscriber sub2 = n2.subscribe("/camera/camera/aligned_depth_to_color/image_raw", 1, depth_callback);
    ros::Subscriber sub3 = n3.subscribe("/camera/camera/aligned_depth_to_color/camera_info", 1, depth_infor_callback);
    ros::spin();
    return 0;
}
void function1(){
    waitKey(0);
}
void function2(){
    lock();
        if(get_image){
            detect_qr();
            get_image=0;
        }
    unlock();
}