#include "../mvibot_marker_init.h"
using namespace  std;
int marker::detect_l(){
    static int value_return;
    value_return=-1;
    //
    static line line3,line4;
    static float dis_error_min;
    dis_error_min=99;
    for(int i=0;i<my_data.my_lines2.size();i++){
        for(int j=1;j<my_data.my_lines2[i].size();j++){
                static float ampha,beta,delta;
                ampha=my_data.my_lines2[i][j-1].caculate_atan2();
                beta=my_data.my_lines2[i][j].caculate_atan2();
                //
                static float dis1,dis2,dis3;
                dis1=my_data.my_lines2[i][j-1].caculate_dis();
                dis2=my_data.my_lines2[i][j].caculate_dis();
                cout<<fabs(fabs(ampha-beta)-M_PI/2)/M_PI*180<<endl;
                if(fabs(fabs(ampha-beta)-M_PI/2)<=M_PI/180*5)
                {
                    cout<<dis1<<"|"<<dis2<<endl;
                    if(dis1 < 0.4+0.05 & dis1 > 0.4 -0.05 & dis2 <0.6+0.05 & dis2 > 0.6-0.05){
                        static float dis_error_sum;
                        dis_error_sum=fabs(0.4-dis1)+fabs(0.6-dis2);
                        if(dis_error_sum < dis_error_min){
                            line3=my_data.my_lines2[i][j-1];
                            line4=my_data.my_lines2[i][j];
                            dis_error_min=dis_error_sum;
                            value_return=1;
                        }
                    }
                }
            }
    }
    if(value_return==1){
        static float x,y;
        static  geometry_msgs::Quaternion quat;
        static point point_1;
        //
        point_1=line_cut(line3,line4);
        cout<<"\t x:"<<point_1.x<<"|y:"<<point_1.y<<endl;
        my_pose.resize(1);
        my_pose[0].position.x=point_1.x;
        my_pose[0].position.y=point_1.y;
        //
        static float beta;
        beta=line3.caculate_atan2();
        if(beta>M_PI_2) beta=beta-M_PI;
        if(beta<-M_PI_2) beta=beta+M_PI;
        quat=tf::createQuaternionMsgFromYaw(beta);
        my_pose[0].orientation=quat;
        //
        static string cmd;
        cmd="";
        cmd=cmd+"echo "+to_string( my_pose[0].position.x)+"\t"+to_string( my_pose[0].position.y)+"\t"+to_string(tf::getYaw(my_pose[0].orientation));
        static struct timespec realtime;
        clock_gettime(CLOCK_REALTIME, &realtime);
        cmd=cmd+" >> /home/mvibot/catkin_ws/src/mvibot/data/l/"+to_string(realtime.tv_sec)+".txt";
        system(cmd.c_str());
        //
        static ofstream myfile;
        myfile.open("/home/mvibot/catkin_ws/src/mvibot/data/l/"+to_string(realtime.tv_sec)+".txt",ios::app);
        for(int i=0;i<my_data.my_points.size();i++){
            cmd=to_string(my_data.my_points[i].x)+"\t"+to_string(my_data.my_points[i].y)+"\n";
            myfile<<cmd;
        }
        myfile.close();
        //
    }
    return value_return;
}