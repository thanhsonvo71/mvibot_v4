#include "../mvibot_marker_init.h"
using namespace  std;
int data::process_data(string marker_dir){
    static int n;
    static int value_return;
    n=my_scan.size();
    if(n>=num_msg){
        value_return=1;
        //
        cout<<"Start process**********************************"<<endl;
        cout<<"Get "<<num_msg<<" msg:Finsh"<<endl;
        //
        cout<<"process point:"<<endl;
        static float theta,x,y;
        for(int i=0;i<my_scan[0].ranges.size();i++){
                theta=my_scan[0].angle_min+i*my_scan[0].angle_increment;
                for(int j=0;j<n;j++){  
                    //
                    static float x_max,x_min;
                    static float y_max=1.0,y_min=-1.0;
                    if(marker_dir=="front_ward") {
                        x_max=2.2; x_min=0.5;
                        x=my_scan[j].ranges[i]*cos(theta);
                        y=my_scan[j].ranges[i]*sin(theta);
                    }else if(marker_dir=="back_ward"){
                        x_max=-0.5; x_min=-2.2;
                        x=-my_scan[j].ranges[i]*cos(theta);
                        y=-my_scan[j].ranges[i]*sin(theta);
                    }
                    //   
                    if(x>=x_min & x<=x_max){
                        if(y>=y_min & y<=y_max){
                            static int size_points;
                            size_points=my_points.size();
                            my_points.resize(size_points+1);
                            my_points[size_points].x=x;
                            my_points[size_points].y=y;
                        }
                    }
                }
            }
        cout<<"\t num_point:"<<my_points.size()<<endl;
        // process group
        // for the rotary cw/ccw of radar. 
        // 2 point continuous in angle if have distane > 0.1 then they are 2 group
        cout<<"process group points:"<<endl;
        for(int i=0;i<my_points.size();i++){
                static int size_points;
                static int size_group;
                static float x,y;
                x=my_points[i].x;
                y=my_points[i].y;
                //
                size_group=my_group.size();
                if(size_group==0) {
                        my_group.resize(1);
                        size_group=1;
                }
                //
                size_points=my_group[size_group-1].points.size();
                if(size_points==0){
                        my_group[size_group-1].points.resize(1);
                        size_points=1;
                        my_group[size_group-1].points[size_points-1].x=x;
                        my_group[size_group-1].points[size_points-1].y=y;
                }else{
                    static float dis_x,dis_y;
                    dis_x=x-my_group[size_group-1].points[size_points-1].x;
                    dis_y=y-my_group[size_group-1].points[size_points-1].y;
                    if(sqrt(dis_x*dis_x+dis_y*dis_y)<=0.1){
                        my_group[size_group-1].points.resize(size_points+1);
                        my_group[size_group-1].points[size_points].x=x;
                        my_group[size_group-1].points[size_points].y=y;
                    }else{
                        my_group.resize(size_group+1);
                        my_group[size_group].points.resize(1);
                        my_group[size_group].points[0].x=x;
                        my_group[size_group].points[0].y=y;
                    }
                }
            }
        cout<<"\t num_group:"<<my_group.size()<<endl;
        // group filter
        // fillter points with neighbor, point filletr by  avegare 10 point around it 
        cout<<"filter group...."<<endl;
        my_group2.resize(my_group.size());
        for(int i=0;i<my_group2.size();i++){
            my_group2[i].points=my_group[i].points;
            if(my_group[i].points.size()>11) 
            {
                static float sum_x,sum_y,n;
                for(int j=5;j<my_group2[i].points.size()-5;j++){
                    sum_x=0;
                    sum_y=0;
                    n=0;
                    for(int k=j-5;k<=j+5;k++){
                        sum_x+=my_group[i].points[k].x;
                        sum_y+=my_group[i].points[k].y;
                        n++;
                    }
                    my_group2[i].points[j].x=sum_x/n;
                    my_group2[i].points[j].y=sum_y/n;
                }
            }
        }
        // group filter with perimeter
        // because with vl , L ,Bar marker always have perimeter > 0.3 then remove if group have perimeter < 0.3
        cout<<"filter group with perimeter...."<<endl;
        my_group3.resize(0);
        for(int i=0;i<my_group2.size();i++){
            static float perimeter;
            perimeter=0;
            if(my_group2[i].points.size()>=2){
                for(int j=1;j<my_group2[i].points.size();j++){
                    static float dis_x,dis_y;
                    dis_x=my_group2[i].points[j].x-my_group2[i].points[j-1].x;
                    dis_y=my_group2[i].points[j].y-my_group2[i].points[j-1].y;
                    perimeter+=sqrt(dis_x*dis_x+dis_y*dis_y);
                }
            }
            if(perimeter>0.3){
                cout<<"\t group:"<<i<<endl;
                cout<<"\t \t size_points:"<<my_group2[i].points.size()<<endl;
                cout<<"\t \t perimeter:"<<perimeter<<endl;
                //
                my_group3.resize(my_group3.size()+1);
                my_group3[my_group3.size()-1]=my_group2[i];
            }
        }
        // detect line detect small line 0.1 & 0.05
        my_lines.resize(0);
        my_lines2.resize(my_group3.size());
        static line line1,line2;
        static float distance_max;
        static int n;
        distance_max=0;
        static int is_break;
        is_break=0;
        for(int i=0;i<my_group3.size();i++){
            for(int j=0;j<my_group3[i].points.size();j++){
                line1.points.resize(0);
                line2.points.resize(0);
                distance_max=0;
                is_break=0;
                for(int k=j;k<my_group3[i].points.size();k++){
                    if(line1.caculate_dis()<0.1){
                        line1.addpoint(my_group3[i].points[k]);
                    }else{
                        line1.caculate();
                        for(int n=0;n<line1.points.size();n++){
                            static float distance;
                            distance=line1.caculate_distance(line1.points[n]);
                            if(distance_max<distance) distance_max=distance;
                        }
                        if(distance_max<0.005){
                            line2.points.resize(0);
                            for(n=k;n<my_group3[i].points.size();n++){
                                if(line2.caculate_dis()<0.05){
                                    line2.addpoint(my_group3[i].points[n]);
                                }else{
                                    for(int m=0;m<line2.points.size();m++){
                                        static float distance;
                                        distance=line1.caculate_distance(line2.points[m]);
                                        k++;
                                        if(distance<0.005) line1.addpoint(line2.points[m]);
                                        else{
                                            static float check_sum;
                                            static int n_check;
                                            n_check=0;
                                            check_sum=0;
                                            for(int h=m;h<line2.points.size();h++){
                                                check_sum+=line1.caculate_distance(line2.points[h]);
                                                n_check++;
                                            }
                                            check_sum=check_sum/n_check;
                                            if(check_sum>=0.005){
                                                is_break=1;
                                                break;
                                            }else line1.addpoint(line2.points[m]);
                                            is_break=1;
                                            break;
                                            
                                        }
                                    }
                                    line2.points.resize(0);
                                    line1.caculate();
                                }
                                if(is_break) {
                                    static int size;
                                    size=my_lines.size();
                                    my_lines.resize(size+1);
                                    my_lines[size]=line1;
                                    //
                                    size=my_lines2[i].size();
                                    my_lines2[i].resize(size+1);
                                    my_lines2[i][size]=line1;
                                    j=k;
                                    break;
                                }
                            }
                        }else{
                            is_break=1;
                            break;
                        }
                        if(is_break) break;
                    }
                }
            }
            //
            static int size;
            size=my_lines.size();
            my_lines.resize(size+1);
            my_lines[size]=line1;
            size=my_lines2[i].size();
            my_lines2[i].resize(size+1);
            my_lines2[i][size]=line1;
        } 
        // combie line if line have as same as 
        for(int i=0;i<my_lines2.size();i++){
            for(int j=1;j<my_lines2[i].size();j++){
                static float ampha,beta,delta;
                ampha=my_lines2[i][j-1].caculate_atan2();
                beta=my_lines2[i][j].caculate_atan2();
                delta=tan(beta-ampha);
                if(fabs(delta)<=M_PI/180*10 | my_lines2[i][j].caculate_dis()<=0.05){
                    for(int k=0;k<my_lines2[i][j].points.size();k++){
                        my_lines2[i][j-1].addpoint(my_lines2[i][j].points[k]);
                    }
                    my_lines2[i][j-1].caculate();
                    for(int k=j;k<my_lines2[i].size()-1;k++){
                        my_lines2[i][k]=my_lines2[i][k+1];
                    }
                    my_lines2[i].resize(my_lines2[i].size()-1);
                    j--;
                }
            }  
        }
        // update line with new point
        for(int i=0;i<my_lines2.size();i++){
            for(int j=0;j<my_lines2[i].size();j++)
            my_lines2[i][j].points.resize(0);
        }
        for(int i=0;i<my_group3.size();i++){
            for(int j=0;j<my_group3[i].points.size();j++){
                for(int k=0;k<my_lines2[i].size();k++){
                    if(my_lines2[i][k].caculate_distance(my_group3[i].points[j])<=0.01)
                    my_lines2[i][k].addpoint(my_group3[i].points[j]);
                }
            }
        }
        for(int i=0;i<my_lines2.size();i++){
            for(int j=0;j<my_lines2[i].size();j++)
            my_lines2[i][j].caculate();
        }
        // get infor line
        cout<<"infor line"<<endl;
        for(int i=0;i<my_lines2.size();i++){
            cout<<"\t group:"<<i<<endl;
            for(int j=0;j<my_lines2[i].size();j++)
            {
                cout<<"\t \t line:"<<j<<endl;
                cout<<"\t \t \t dis:"<<my_lines2[i][j].caculate_dis()<<endl;
                cout<<"\t \t \t angle:"<<my_lines2[i][j].caculate_atan2()/M_PI*180<<endl;
            }
        }
        //
    }else {
        cout<<"Not enough data scan->get more scan data"<<endl;
        value_return=0;
    }
    return value_return;
}
void data::reset(){
        my_scan.resize(0);
        my_points.resize(0);
        my_group.resize(0);
        my_group2.resize(0);
        my_group3.resize(0);
        my_lines.resize(0);
        my_lines2.resize(0);
        my_lines2_1.resize(0);
        my_lines3.resize(0);
}