#include "../mvibot_marker_init.h"
#include "../../common/send_tranfrom/send_tranfrom.h"
#include "../../common/get_position/get_position.h"
using namespace  std;
int marker::caculate_transforms_ofset(){
    static int value_return;
    value_return=0;
    if(marker_type=="none_marker_dis" | marker_type=="none_marker_angle" ){
        // creat pose for marker dis or angle
        my_pose.resize(1);
        if(marker_type=="none_marker_dis"){
            my_pose[0].position.x=off_set_dis;
            my_pose[0].position.y=0;
            my_pose[0].position.z=0;
            //
            my_pose[0].orientation.x=0;
            my_pose[0].orientation.y=0;
            my_pose[0].orientation.z=0;
            my_pose[0].orientation.w=1;
        }else if(marker_type=="none_marker_angle"){
            my_pose[0].position.x=0;
            my_pose[0].position.y=0;
            my_pose[0].position.z=0;
            //
            static  geometry_msgs::Quaternion quat;
            quat=tf::createQuaternionMsgFromYaw(off_set_angle/180*M_PI);
            my_pose[0].orientation=quat;
        }
        // caculator tranfom
        static double theta;
        my_pose2.resize(2);
        theta=getyaw(my_pose[0].orientation);
        // origin pose (pose o)
        my_pose2[0]=my_pose[0];
        // tranfrom pose (pose n)
        my_pose2[1]=my_pose[0];
        //        
    }else{
        static double theta0, theta1;
        my_pose2.resize(2);
        theta0=getyaw(my_pose[0].orientation);
        if(theta0>M_PI/2) theta0=theta0-M_PI;
        if(theta0<-M_PI/2) theta0=theta0+M_PI;
        // caculator tranfom
        my_pose2[0]=my_pose[0];
        double x0,y0,x1,y1,xn,yn;
        x0=my_pose2[0].position.x;
        y0=my_pose2[0].position.y;
        x1=x0+off_set_x*sin(M_PI/2-theta0);
        y1=y0+off_set_x*cos(M_PI/2-theta0);
        xn=x1-off_set_y*sin(theta0);
        yn=y1+off_set_y*cos(theta0);
        //
        my_pose2[1]=my_pose2[0];
        my_pose2[1].position.x=xn;
        my_pose2[1].position.y=yn;
    }
    // check position robot with frame odom
    if(robot_position[0]!=-1 | robot_position[1]!=-1 | robot_position[2]!=-1 | robot_position[3]!=-1){
        value_return=1;
        x_set=robot_position[0];
        y_set=robot_position[1];
        z_set=robot_position[2];
        w_set=robot_position[3];
        cout<<x_set<<"|"<<y_set<<"|"<<z_set<<"|"<<w_set<<endl;
        //
        new_update=1;
    }
    else{
        value_return=0;
        cout<<"Robot position not have !"<<endl;
    }
    return value_return;
}
int marker::check_send_transforms_tf_frame(){
    static int value_return;
    // check tranfrom is true
    robot_position_get=get_position2(mvibot_seri+"/odom",mvibot_seri+"/base_marker");
    // update time get time
    t_get.sec=(uint32_t)robot_position_get[4];
    t_get.nsec=(uint32_t)robot_position_get[5];
    //
    value_return=0;
    if(t_get>t_send+ros::Duration(0.2)) //0.1->0.2
    {
        if(compare_pose(x_set,y_set,z_set,w_set,robot_position_get[0],robot_position_get[1],robot_position_get[2],robot_position_get[3],0.05000,0.05000)) 
        value_return=1;
    }
    return value_return;
}
int marker::tranfrom_pose_marker(int mode){
    static int value_return;
    static tf::TransformListener listener;
    static tf::StampedTransform transform;
    value_return=0;
    try{
        try {  
            pose_o.header.stamp=ros::Time(0);
            pose_n.header.stamp=ros::Time(0);
            listener.waitForTransform(mvibot_seri+"/odom", mvibot_seri+"/base_marker", ros::Time(0), ros::Duration(0.1));
            listener.lookupTransform (mvibot_seri+"/odom", mvibot_seri+"/base_marker", ros::Time(0), transform);
            if(mode==1){
                //
                pose_o.pose=my_pose2[0];
                pose_n.pose=my_pose2[1];
                //
                pose_o.header.frame_id=mvibot_seri+"/base_marker";
                pose_n.header.frame_id=mvibot_seri+"/base_marker";
                //
                listener.transformPose(mvibot_seri+"/base_footprint",pose_o,pose_o_robot);
                listener.transformPose(mvibot_seri+"/base_footprint",pose_n,pose_n_robot);
            }
            value_return=1;
        } catch(ros::Exception &e){
            ROS_ERROR("Error occured1: %s ", e.what());
            value_return=0;
        }
    }catch(const std::exception& e){
            ROS_ERROR("Error occured2: %s ", e.what());
            value_return=0;
    }
    return value_return;
}
int marker::tranfrom_pose_marker2(int mode){
    static int value_return;
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    static geometry_msgs::TransformStamped target;
    value_return=0;
    try{
        try {             
            target=tfBuffer.lookupTransform(mvibot_seri+"/base_footprint", mvibot_seri+"/base_marker", ros::Time(0), ros::Duration(0.1));
            if(mode==1){
                pose_o.header.stamp=target.header.stamp;
                pose_n.header.stamp=target.header.stamp;
                //
                pose_o.pose=my_pose2[0];
                pose_n.pose=my_pose2[1];
                //
                pose_o.header.frame_id=mvibot_seri+"/base_marker";
                pose_n.header.frame_id=mvibot_seri+"/base_marker";
                //
                tf2::doTransform(pose_o, pose_o_robot, target);
                tf2::doTransform(pose_n, pose_n_robot, target);
            }
            value_return=1;
        } catch(ros::Exception &e){
            ROS_ERROR("Error occured1: %s ", e.what());
            value_return=0;
        }
    }catch(const std::exception& e){
            ROS_ERROR("Error occured2: %s ", e.what());
            value_return=0;
    }
    return value_return;
}
//
int marker::check_first_tranfrom_pose_marker(){
    static int value_return;
    static double xo1,yo1,xo2,yo2;
    static double zo1,wo1,zo2,wo2;
    //
    xo1=pose_o.pose.position.x;
    yo1=pose_o.pose.position.y;
    zo1=pose_o.pose.orientation.z;
    wo1=pose_o.pose.orientation.w;
    //
    xo2=pose_o_robot.pose.position.x;
    yo2=pose_o_robot.pose.position.y;
    zo2=pose_o_robot.pose.orientation.z;
    wo2=pose_o_robot.pose.orientation.w;   
    //
    value_return=0;
    if(compare_pose(xo1,yo1,zo1,wo1,xo2,yo2,zo2,wo2,0.05000,0.05000)) value_return=1;
    else value_return=0;
    return value_return;
}
int marker::move_to_postion_pose_n(){
    static int value_return;
    value_return=0;
    cout<<"move to postion pose_n:"<<endl;
    cout<<"\t _x:"<<pose_n_robot.pose.position.x;
    cout<<"|_y:"<<pose_n_robot.pose.position.y;
    cout<<"|_theta:"<<atan2(pose_n_robot.pose.position.y,pose_n_robot.pose.position.x)/M_PI*180<<endl;
    //
    static double dis,angle;
    static double x,y;
    x=pose_n_robot.pose.position.x;
    y=pose_n_robot.pose.position.y;
    if(marker_type=="none_marker_dis"){
        angle=getyaw(pose_n_robot.pose.orientation);
        dis=fabs(x);
        off_set_dis=x;
    }
    else{
        angle=atan2(y,x);
        dis=sqrt(x*x+y*y);
    }
    //
    if(angle>M_PI*1/2) angle=angle-M_PI;
    if(angle<-M_PI_2*1/2) angle=angle+M_PI;
    //
    static float v,w;
    v=0;
    w=0;
    //
    if(fabs(dis)<=0.01){ // 0.005
        value_return=1;
        v=0;
        w=0;
        pub_cmd_vel(0,0);
        robot_emg();
    }else{
        if(fabs(angle)<=M_PI/180*30){
            static int k;
            if(x>0) k=1;
            if(x<0) k=-1;
            //
            if(fabs(dis)>=0.6)  v=0.3*k;
            else if(fabs(dis)<0.6 & fabs(dis)>=0.4) v=0.25*k;
            else if(fabs(dis)<0.4 & fabs(dis)>=0.2) v=0.15*k;
            else if(fabs(dis)<0.2 & fabs(dis)>=0.1) v=0.08*k;
            else if(fabs(dis)<0.1 & fabs(dis)>=0.05) v=0.04*k;
            else if(fabs(dis)<0.05 & fabs(dis)>=0.03) v=0.02*k;
            else if(fabs(dis)<=0.03) v=0.02*k;
            //
            static int k2;
            if(angle>0) k2=1;
            if(angle<0) k2=-1;
            //
            if(fabs(angle)>=M_PI/180*15) w=k2*M_PI/180*10;	    
            else if( fabs(angle)<M_PI/180*15 &  fabs(angle)>=M_PI/180*10) w=k2*M_PI/180*5;	    
            else if( fabs(angle)<M_PI/180*10 &  fabs(angle)>=M_PI/180*5) w=k2*M_PI/180*5;	    
            else if( fabs(angle)<M_PI/180*5 &  fabs(angle)>=M_PI/180*3) w=k2*M_PI/180*3;
            else if( fabs(angle)<M_PI/180*3 &  fabs(angle)>=M_PI/180/2) w=k2*M_PI/180*1;
            else if( fabs(angle)<M_PI/180/2) w=k2*M_PI/180/2;
        }
        else{
            v=0;
            if(angle>0) w=M_PI/10;//180*10;
            else w=-M_PI/10;
        }
    }
    cout<<"v:"<<v<<"|w"<<w<<endl;
    //
    if(safe!=0){
        robot_emg();
        pub_cmd_vel(0,0);
    }else pub_cmd_vel(v,w);
    return value_return;
}
int marker::move_to_orientation_pose_n(){
    static int value_return;
    value_return=0;
    cout<<"move to orientation pose_n:"<<endl;
    cout<<"_theta:"<<getyaw(pose_n_robot.pose.orientation)/M_PI*180<<endl;
    //
    static double dis,angle;
    static double x,y;
    angle=getyaw(pose_n_robot.pose.orientation);
    //
    if(marker_type=="none_marker_angle") off_set_angle=angle/M_PI*180;
    static float v,w;
    v=0;
    w=0;
    if(fabs(angle)<=M_PI/180*30){
            if(fabs(angle)<=M_PI/180/3){
                value_return=1;
                v=0;
                w=0;
                robot_emg();
                pub_cmd_vel(0,0);
            }else{
                static int k2;
                if(angle>0) k2=1;
                if(angle<0) k2=-1;
                //
                if(fabs(angle)>=M_PI/180*15) w=k2*M_PI/180*10;	    
                else if( fabs(angle)<M_PI/180*15 &  fabs(angle)>=M_PI/180*10) w=k2*M_PI/180*5;	    
                else if( fabs(angle)<M_PI/180*10 &  fabs(angle)>=M_PI/180*5) w=k2*M_PI/180*5;	    
                else if( fabs(angle)<M_PI/180*5 &  fabs(angle)>=M_PI/180*3) w=k2*M_PI/180*3;
                else if( fabs(angle)<M_PI/180*3 &  fabs(angle)>=M_PI/180/2) w=k2*M_PI/180*1;
                else if( fabs(angle)<M_PI/180/2) w=k2*M_PI/180/2;
            } 
    }
    else{
            if(angle>0) w=M_PI/10;
            else w=-M_PI/10;
    }
    cout<<"v:"<<v<<"|w"<<w<<endl;
    //
    if(safe!=0){
        robot_emg();
        pub_cmd_vel(0,0);
    }else pub_cmd_vel(v,w);
    return value_return;
}
int marker::action(){
    static int value_return;
    static int res;
    //
    if(status==0){
        cout<<"Collect data..."<<endl;
        if(marker_type=="none_marker_dis" |  marker_type=="none_marker_angle"){
            status=1;
            cout<<"Finsish collect data..."<<endl;
        }
        else{
            if(my_data.process_data(marker_dir)){
                status=1;
                cout<<"Finsish collect data..."<<endl;
            }
        }
    }
    else if(status==1){
        if(marker_type=="none_marker_dis" |  marker_type=="none_marker_angle"){
            status=2;
            cout<<"don 't near detect so skip"<<endl;
        }else{
            // detect maker
            res=1;
            if(marker_type=="vl_marker")    res=detect_vl();
            if(marker_type=="bar_marker")   res=detect_bar(bar_distance);
            if(marker_type=="l_marker")     res=detect_l();
            //
            if(res==1) {
                cout<<"\t Detect finsh"<<endl;
                cout<<"\t Continue caculate tranform offset...."<<endl;
                status=2;
            }
            else{
                my_data.reset();
                status=0;
                cout<<"Can't detect. Retry collet laser msg...."<<endl;
            }
        }
    }
    else if(status==2){
        res=caculate_transforms_ofset();
        if(res==1) status=3;
        cout<<"Cacluate transfrom offset finish"<<endl;
    }
    else if(status>=3){
        send_tranfrom2(x_set,y_set,z_set,w_set,mvibot_seri+"/odom",mvibot_seri+"/base_marker");
        if(new_update==1){
            new_update=0;
            t_send=ros::Time::now();
        }
        //
        if(status==3){
            cout<<"Check is send transform odom->base_marker"<<endl;
            res=check_send_transforms_tf_frame();
            if(res==1) status=4;
        }
        else if(status>=4){
            static int status_transfrom_pose;
            status_transfrom_pose=tranfrom_pose_marker2(1);
            if(status==4){
                cout<<"Check first pose is match with position robot!"<<endl;
                if(status_transfrom_pose==1){
                    if(check_first_tranfrom_pose_marker()){
                        cout<<"First pose is match with postion robot"<<endl;
                        status=5;
                    }
                }
            }
            else if(status==5){
                cout<<"Get footprint robot!"<<endl;
                if(get_footprint()) {
                    cout<<"Finish get footprint robot!"<<endl;                  
                    status=6;
                    active_step=0;
                }
            }
            else if(status==6){
                cout<<"Action move!"<<endl;
                if(status_transfrom_pose){
                    // safe
                    if(safe==0) {
                        if(check_safe()==1)  safe=30;
                    }
                    else{
                        if(check_safe()==0) safe--;
                        if(safe<0) safe=0;
                    }
                    //
                    if(active_step==0) res=move_to_postion_pose_n();
                    if(active_step==1) res=move_to_orientation_pose_n();
                    //
                    if(res==1){
                        active_step++;
                        if(active_step>=2){
                            active_step=0;
                            status=0;
                            reset(0);
                            start=2;
                            pub_cmd_vel(0,0);
                            robot_emg();
                            //
                            off_set_x=0.0;
                            off_set_y=0.0;
                            off_set_dis=0.0;
                            off_set_angle=0.0;
                            marker_data="";
                            //
                            cout<<"Finish marker"<<endl;
                        }
                    }
                }
            }
        }
    }
    return value_return;
}
void marker::reset(int mode){
    if(mode==0){
        my_data.reset();
        process_data(marker_data);
        status=0;
        active_step=0;
        new_update=0;
    }else if(mode==1){
        my_data.reset();
        status=0;
        active_step=0;
        new_update=0;
    }
   
}
