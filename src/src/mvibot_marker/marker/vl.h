#include "../mvibot_marker_init.h"
using namespace  std;
int marker::detect_vl(){
    static int value_return;
    value_return=-1;
    static float ampha1;
    // vl detect
    static int num_group;
    static line line1,line2,line3;
    for(int i=0;i<my_data.my_lines2.size();i++){
        for(int j=1;j<my_data.my_lines2[i].size()-1;j++){
                static float ampha,beta,delta;
                ampha1=ampha;
                beta=my_data.my_lines2[i][j].caculate_atan2();
                delta=my_data.my_lines2[i][j+1].caculate_atan2();
                //
                static float dis1,dis2,dis3;
                dis1=my_data.my_lines2[i][j-1].caculate_dis();
                dis2=my_data.my_lines2[i][j].caculate_dis();
                dis3=my_data.my_lines2[i][j+1].caculate_dis();
                if(fabs(fabs(delta-beta)-2*M_PI/3)<=M_PI/180*10 | fabs(fabs(delta-beta)-M_PI/3)<=M_PI/180*10)
                {
                    if(fabs(fabs(beta-ampha)-5*M_PI/6)< M_PI/180*10 | fabs(fabs(beta-ampha)-1*M_PI/6)< M_PI/180*10){
                        if(dis1>=0.35 &  dis2 >=0.15 & dis2 <=0.2 &  dis3>=0.15 & dis3 <=0.2){ 
                            value_return=1;
                            num_group=i;
                            line1=my_data.my_lines2[i][j-1];
                            line2=my_data.my_lines2[i][j];
                            line3=my_data.my_lines2[i][j+1];
                            //
                            line1.points.resize(0);
                            line2.points.resize(0);
                            line3.points.resize(0);
                            //
                            for(int k=0;k<my_data.my_group3[i].points.size();k++){
                                if(my_data.my_lines2[i][j-1].caculate_distance(my_data.my_group3[i].points[k]) <= 0.02) line1.addpoint(my_data.my_group3[i].points[k]);
                                if(my_data.my_lines2[i][j].caculate_distance(my_data.my_group3[i].points[k]) <= 0.02) line2.addpoint(my_data.my_group3[i].points[k]);
                                if(my_data.my_lines2[i][j+1].caculate_distance(my_data.my_group3[i].points[k]) <= 0.02) line3.addpoint(my_data.my_group3[i].points[k]);
                            }
                            line1.caculate();
                            line2.caculate();
                            line3.caculate();
                        }
                    }
                }
            }
    }
    if(value_return==1){
        static point point_1,point_2,point_3;
        //
        point_1.x=0;
        point_1.y=0;
        for(int i=0;i<line1.points.size();i++){
            point_1.x+=line1.points[i].x;
            point_1.y+=line1.points[i].y;
        }
        point_1.x=point_1.x/line1.points.size();
        point_1.y=point_1.y/line1.points.size();
        static  line line1_2;
        line1_2.points.resize(0);
        for(int i=0;i<line1.points.size();i++){
            static float dis_x,dis_y;
            dis_x=point_1.x-line1.points[i].x;
            dis_y=point_1.y-line1.points[i].y;
            if(sqrt(dis_x*dis_x+dis_y*dis_y)<=0.1) line1_2.addpoint(line1.points[i]);
        }
        line1_2.caculate();
        //
        point_2.x=0;
        point_2.y=0;
        for(int i=0;i<line2.points.size();i++){
            point_2.x+=line2.points[i].x;
            point_2.y+=line2.points[i].y;
        }
        point_2.x=point_2.x/line2.points.size();
        point_2.y=point_2.y/line2.points.size();
        static  line line2_2;
        line2_2.points.resize(0);
        for(int i=0;i<line2.points.size();i++){
            static float dis_x,dis_y;
            dis_x=point_2.x-line2.points[i].x;
            dis_y=point_2.y-line2.points[i].y;
            if(sqrt(dis_x*dis_x+dis_y*dis_y)<=0.05) line2_2.addpoint(line2.points[i]);
        }
        line2_2.caculate();
        //
         point_3.x=0;
         point_3.y=0;
        for(int i=0;i<line3.points.size();i++){
             point_3.x+=line3.points[i].x;
             point_3.y+=line3.points[i].y;
        }
        point_3.x= point_3.x/line3.points.size();
        point_3.y= point_3.y/line3.points.size();
        static  line line3_2;
        line3_2.points.resize(0);
        for(int i=0;i<line3.points.size();i++){
            static float dis_x,dis_y;
            dis_x= point_3.x-line3.points[i].x;
            dis_y= point_3.y-line3.points[i].y;
            if(sqrt(dis_x*dis_x+dis_y*dis_y)<=0.05) line3_2.addpoint(line3.points[i]);
        }
        line3_2.caculate();
        //
        //cout<<line1_2.caculate_atan2()/M_PI*180<<"A"<<line2_2.caculate_atan2()/M_PI*180<<"A"<<line3_2.caculate_atan2()/M_PI*180<<"A"<<endl;
        my_data.my_lines2[num_group].resize(3);
        my_data.my_lines2[num_group][0]=line1_2;
        my_data.my_lines2[num_group][1]=line2_2;
        my_data.my_lines2[num_group][2]=line3_2;
        //
        static point point_o;
        point_o=line_cut(line1_2,line2_2);
        cout<<"\t x:"<<point_o.x<<"|y:"<<point_o.y<<endl;
        static point point_o2,point_o3;
        point_o2=line_cut(line2_2,line3_2);
        point_o3=line_cut(line3_2,line1_2);
        //
        static float dis_1_2,dis_3_2;
        dis_1_2=sqrt((point_o.x-point_o2.x)*(point_o.x-point_o2.x)+(point_o.y-point_o2.y)*(point_o.y-point_o2.y));
        dis_3_2=sqrt((point_o3.x-point_o2.x)*(point_o3.x-point_o2.x)+(point_o3.y-point_o2.y)*(point_o3.y-point_o2.y));
        cout<<dis_1_2<<"|"<<dis_3_2<<endl;
        if(fabs(dis_1_2-0.15)<=0.03){
                my_pose.resize(1);
                my_pose[0].position.x=point_o.x;
                my_pose[0].position.y=point_o.y;
                ampha1=line1_2.caculate_atan2();
                static  geometry_msgs::Quaternion quat;
                if(ampha1>M_PI_2) ampha1=ampha1-M_PI;
                if(ampha1<-M_PI_2) ampha1=ampha1+M_PI;
                quat=tf::createQuaternionMsgFromYaw(ampha1);
                my_pose[0].orientation=quat;
                // save data to txt
                static string cmd;
                cmd="";
                cmd=cmd+"echo "+to_string( my_pose[0].position.x)+"\t"+to_string( my_pose[0].position.y)+"\t"+to_string(tf::getYaw(my_pose[0].orientation));
                static struct timespec realtime;
                clock_gettime(CLOCK_REALTIME, &realtime);
                cmd=cmd+" >> /home/mvibot/catkin_ws/src/mvibot/data/vl/"+to_string(realtime.tv_sec)+".txt";
                system(cmd.c_str());
                //
                static ofstream myfile;
                myfile.open("/home/mvibot/catkin_ws/src/mvibot/data/vl/"+to_string(realtime.tv_sec)+".txt",ios::app);
                for(int i=0;i<my_data.my_points.size();i++){
                    cmd=to_string(my_data.my_points[i].x)+"\t"+to_string(my_data.my_points[i].y)+"\n";
                    myfile<<cmd;
                }
                myfile.close();
                //
        }else value_return=0;
        //
    }
    //value_return=0;
    return value_return;
}