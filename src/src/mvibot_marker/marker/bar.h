#include "../mvibot_marker_init.h"
using namespace  std;
int marker::detect_bar(float bar_distance){
    static int value_return;
    static vector<vector<line>> my_lines_group_memory;
    static vector<line> my_lines_memory;
    //
    value_return=-1;
    //bar_distance=0.785;
    //
    my_lines_group_memory.resize(0);
    for(int i=0;i<my_data.my_lines2.size();i++){
        for(int j=0;j<my_data.my_lines2[i].size();j++){
            for(int k=i+1;k<my_data.my_lines2.size();k++){
                for(int h=0;h<my_data.my_lines2[k].size();h++){
                    static float ampha,beta,delta;
                    ampha=my_data.my_lines2[i][j].caculate_atan2();
                    beta=my_data.my_lines2[k][h].caculate_atan2();
                    delta=beta-ampha;
                    // check delta //
                    if(fabs(delta)<M_PI/180*2)   
                    {
                        static float dis1,dis2;
                        dis1=my_data.my_lines2[i][j].caculate_dis();
                        dis2=my_data.my_lines2[k][h].caculate_dis();
                        if((dis1>=0.5 | dis2>=0.5) & dis1 >=0.15 & dis2 >=0.15){
                            my_lines_group_memory.resize(my_lines_group_memory.size()+1);
                            my_lines_group_memory[my_lines_group_memory.size()-1].resize(2);
                            my_lines_group_memory[my_lines_group_memory.size()-1][0]=my_data.my_lines2[i][j];
                            my_lines_group_memory[my_lines_group_memory.size()-1][1]=my_data.my_lines2[k][h];
                        }
                    }
                }
            }
        }
    }
    cout<<"Num bar group have:"<<my_lines_group_memory.size()<<endl;
    cout<<"Check bar group with distance_bar_set:"<<bar_distance<<endl;
    static int num_bar_pass;
    static point point_1,point_2;
    num_bar_pass=0;
    for(int k=0;k<my_lines_group_memory.size();k++){
        point_1.x=0;
        point_1.y=0;
        for(int i=0;i<my_lines_group_memory[k][0].points.size();i++){
            point_1.x+=my_lines_group_memory[k][0].points[i].x;
            point_1.y+=my_lines_group_memory[k][0].points[i].y;
        }
        point_1.x=point_1.x/my_lines_group_memory[k][0].points.size();
        point_1.y=point_1.y/my_lines_group_memory[k][0].points.size();
        //
        point_2.x=0;
        point_2.y=0;
        for(int i=0;i<my_lines_group_memory[k][1].points.size();i++){
            point_2.x+=my_lines_group_memory[k][1].points[i].x;
            point_2.y+=my_lines_group_memory[k][1].points[i].y;
        }
        point_2.x=point_2.x/my_lines_group_memory[k][1].points.size();
        point_2.y=point_2.y/my_lines_group_memory[k][1].points.size();
        //
        static float dis1,dis2;
        dis1=my_lines_group_memory[k][0].caculate_distance(point_2);
        dis2=my_lines_group_memory[k][1].caculate_distance(point_1);
        if(fabs(dis1-bar_distance)<=0.05 & fabs(dis2-bar_distance)<=0.05){
            num_bar_pass++;
            my_lines_memory.resize(2);
            my_lines_memory[0]=my_lines_group_memory[k][0];
            my_lines_memory[1]=my_lines_group_memory[k][1];
        }
    }
    cout<<num_bar_pass<<endl;
    if(my_lines_memory.size()!=0 & num_bar_pass==1){
            my_lines_memory[0].points.resize(0);
            my_lines_memory[1].points.resize(0);
            for(int i=0;i<my_data.my_points.size();i++){
                //
                if(my_lines_memory[0].caculate_distance(my_data.my_points[i])<=0.01){
                    my_lines_memory[0].addpoint(my_data.my_points[i]);
                }
                //
                if(my_lines_memory[1].caculate_distance(my_data.my_points[i])<=0.01){
                    my_lines_memory[1].addpoint(my_data.my_points[i]);
                }
            }
            my_lines_memory[0].caculate();
            my_lines_memory[1].caculate();
            //
            static float dis1_to_o;
            static point point_1s;
            dis1_to_o=999;
            point_1.x=0;
            point_1.y=0;
            for(int i=0;i<my_lines_memory[0].points.size();i++){
                point_1.x+=my_lines_memory[0].points[i].x;
                point_1.y+=my_lines_memory[0].points[i].y;
                //
                static float x_to_o,y_to_o,dis_to_o;
                x_to_o=my_lines_memory[0].points[i].x;
                y_to_o=my_lines_memory[0].points[i].y;
                dis_to_o=sqrt(x_to_o*x_to_o+y_to_o*y_to_o);
                if(dis_to_o<dis1_to_o){
                    dis1_to_o=dis_to_o;
                    point_1s=my_lines_memory[0].points[i];
                }
            }
            point_1.x=point_1.x/my_lines_memory[0].points.size();
            point_1.y=point_1.y/my_lines_memory[0].points.size();
            //
            static float dis2_to_o;
            static point point_2s;
            dis2_to_o=999;
            point_2.x=0;
            point_2.y=0;
            for(int i=0;i<my_lines_memory[1].points.size();i++){
                point_2.x+=my_lines_memory[1].points[i].x;
                point_2.y+=my_lines_memory[1].points[i].y;
                //
                static float x_to_o,y_to_o,dis_to_o;
                x_to_o=my_lines_memory[1].points[i].x;
                y_to_o=my_lines_memory[1].points[i].y;
                dis_to_o=sqrt(x_to_o*x_to_o+y_to_o*y_to_o);
                if(dis_to_o<dis2_to_o){
                    dis2_to_o=dis_to_o;
                    point_2s=my_lines_memory[1].points[i];
                }
            }
            point_2.x=point_2.x/my_lines_memory[1].points.size();
            point_2.y=point_2.y/my_lines_memory[1].points.size();
            //
            static line line1;
            line1.A=tan(my_lines_memory[1].caculate_atan2());
            line1.B=-1;
            line1.C=-point_1.x*line1.A-point_1.y*line1.B;
            static point point_1d;
            point_1d=line_cut(line1,my_lines_memory[1]);
            //
            static line line2;
            line2.A=tan(my_lines_memory[0].caculate_atan2());
            line2.B=-1;
            line2.C=-point_2.x*line2.A-point_2.y*line2.B;
            static point point_2d;
            point_2d=line_cut(line2,my_lines_memory[0]);
            //
            static line line3;
            static float xo_1,yo_1;
            xo_1=(point_1.x+point_2.x)/2;
            yo_1=(point_1.y+point_2.y)/2;
            line3.A=tan((my_lines_memory[0].caculate_atan2()+my_lines_memory[0].caculate_atan2())/2);
            line3.B=-1;
            line3.C=-xo_1*line3.A-yo_1*line3.B;
            //
            static line line4;
            line4.A=tan((my_lines_memory[0].caculate_atan2()+my_lines_memory[0].caculate_atan2())/2);
            line4.B=-1;
            line4.C=-point_1s.x*line4.A-point_1s.y*line4.B;
            //
            static line line5;
            line5.A=tan((my_lines_memory[0].caculate_atan2()+my_lines_memory[0].caculate_atan2())/2);
            line5.B=-1;
            line5.C=-point_2s.x*line5.A-point_2s.y*line5.B;
            //
            static line line6;
            line6.A=tan((my_lines_memory[0].caculate_atan2()+my_lines_memory[0].caculate_atan2())/2+M_PI_2);
            line6.B=-1;
            line6.C=-xo_1*line6.A-yo_1*line6.B;
            //
            static float theta,x,y;
            static point point_o1,point_o2;
            point_o1=line_cut(line6,line4);
            point_o2=line_cut(line6,line5);
            //
            cout<<point_o1.x<<"|"<<point_o1.y<<endl;
            theta=(my_lines_memory[0].caculate_atan2()+my_lines_memory[0].caculate_atan2())/2+M_PI_2;
            if(sqrt(point_o1.x*point_o1.x+point_o1.y*point_o1.y)<sqrt(point_o2.x*point_o2.x+point_o2.y*point_o2.y))
            {
                x=point_o1.x;
                y=point_o1.y;
            }else{
                x=point_o2.x;
                y=point_o2.y;
            }        
            if(theta>M_PI_2) theta=theta-M_PI;
            if(theta<-M_PI_2) theta=theta+M_PI;
            //
            static  geometry_msgs::Quaternion quat;
            quat=tf::createQuaternionMsgFromYaw(theta);
            my_pose.resize(3);
            my_pose[0].position.x=(double)x;
            my_pose[0].position.y=(double)y;
            my_pose[0].orientation=quat;
            my_pose[1].position.x=(double)point_1s.x;
            my_pose[1].position.y=(double)point_1s.y;
            my_pose[1].orientation=quat;
            my_pose[2].position.x=(double)point_2s.x;
            my_pose[2].position.y=(double)point_2s.y;
            my_pose[2].orientation=quat;
            //
            my_data.my_lines2.resize(1);
            my_data.my_lines2[0].resize(2);
            my_data.my_lines2[0][0]=my_lines_memory[0];
            my_data.my_lines2[0][1]=my_lines_memory[1];
            value_return=1;
            //
            static string cmd;
            cmd="";
            cmd=cmd+"echo "+to_string( my_pose[0].position.x)+"\t"+to_string( my_pose[0].position.y)+"\t"+to_string(tf::getYaw(my_pose[0].orientation));
            static struct timespec realtime;
            clock_gettime(CLOCK_REALTIME, &realtime);
            cmd=cmd+" >> /home/mvibot/catkin_ws/src/mvibot/data/bar/"+to_string(realtime.tv_sec)+".txt";
            system(cmd.c_str());
            static ofstream myfile;
            myfile.open("/home/mvibot/catkin_ws/src/mvibot/data/bar/"+to_string(realtime.tv_sec)+".txt",ios::app);
            for(int i=0;i<my_data.my_points.size();i++){
                cmd=to_string(my_data.my_points[i].x)+"\t"+to_string(my_data.my_points[i].y)+"\n";
                myfile<<cmd;
            }
            myfile.close();
            
    } 
    return value_return;
}