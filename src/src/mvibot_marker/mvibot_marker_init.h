#include "../common/libary/libary_basic.h"
#include "../common/libary/libary_ros.h"
#include "../common/string_Iv2/string_Iv2.h"
//
#if !defined(mvibot_marker_define)
    string mvibot_seri;
    // support get yaw
    double getyaw(geometry_msgs::Quaternion quat){
        return tf::getYaw(quat);
    }
    double getyaw2(double data3, double data4){
        geometry_msgs::Quaternion quat_msg;
        quat_msg.x=0;
        quat_msg.y=0;
        quat_msg.z=data3;
        quat_msg.w=data4;
        return tf::getYaw(quat_msg);
    }
    int compare_pose(double x1, double y1, double z1, double w1, double x2, double y2, double z2, double w2, double thresold_position, double thresold_angle){
        geometry_msgs::Pose pose_1,pose_2;
        //
        cout<<x1<<"|"<<y1<<"|"<<z1<<"|"<<w1<<endl;
        cout<<x2<<"|"<<y2<<"|"<<z2<<"|"<<w2<<endl;       
        if(sqrt(pow(x2-x1,2)+pow(y2-y1,2))<=thresold_position){
            if(sqrt(pow(z2-z1,2)+pow(w2-w1,2))<=thresold_angle){
                return 1;
            }   
        }
        return 0;
    }
    // define point
    class point{
    public:
        float x;
        float y;
    };
    // define multiple point
    class points {
        public:
            vector<point> points;
    };
    // define line
    class line{
        public:
            float x1;
            float x2;
            float y1;
            float y2;
            float A;
            float B;
            float C;
            vector<point> points;
            void addpoint(point point_add){
                static int size;
                size=points.size();
                points.resize(size+1);
                points[size]=point_add;
            }
            void caculate(){
                static int size;
                size=points.size();
                if(size>=2){
                    static float sum_1,sum_2;
                    sum_1=0; sum_2=0;
                    for(int i=0;i<size;i++){
                        sum_1+=points[i].x;
                        sum_2+=points[i].y;
                    }
                    //
                    static float x_,y_;
                    x_=sum_1/size;
                    y_=sum_2/size;
                    //
                    sum_1=0; sum_2=0;
                    for(int i=0;i<size;i++){
                        sum_1+=(points[i].x-x_)*(points[i].y-y_);
                        sum_2+=(points[i].x-x_)*(points[i].x-x_);
                    }
                    static float a,b;
                    b=sum_1/sum_2;
                    a=y_-b*x_;
                    A=b;
                    B=-1;
                    C=a;
                }
            }
            float caculate_dis(){
                static int size;
                size=points.size();
                if(size>=2){
                    static float dis,dis_x,dis_y;
                    dis_x=points[0].x-points[size-1].x;
                    dis_y=points[0].y-points[size-1].y;
                    dis=sqrt(dis_x*dis_x+dis_y*dis_y);
                    return dis;
                }else return 0;
            }
            float caculate_distance(point point_caculate){
                static float dis;
                static float x,y;
                x=point_caculate.x;
                y=point_caculate.y;
                dis=fabs(A*x+B*y+C)/sqrt(A*A+B*B);
                return dis;
            }
            float caculate_atan2(){
                return atan2(B,A);
            }
    };
    // define point by line intersect line
    point line_cut(line line1, line line2){
        static float d,dx,dy;
        static float a1,a2,b1,b2,c1,c2;
        //
        a1=line1.A;
        b1=line1.B;
        c1=line1.C;
        a2=line2.A;
        b2=line2.B;
        c2=line2.C;
        d=a1*b2-a2*b1;
        dx=c1*b2-c2*b1;
        dy=a1*c2-a2*c1;            
        //
        static point point_return;
        point_return.x=-dx/d;
        point_return.y=-dy/d;
        //
        return point_return;
    }
    // data for robot
    class data{
        public:
            int num_msg=5; //5
            // data
            vector<sensor_msgs::LaserScan> my_scan;
            vector<point>   my_points;
            vector<points>  my_group;
            vector<points>  my_group2;
            vector<points>  my_group3;
            // line & pose after detect
            vector<line>            my_lines;
            vector<vector<line>>    my_lines2; 
            vector<vector<line>>    my_lines2_1;
            vector<line>            my_lines3; 
            //
            int process_data(string marker_dir);
            void reset();
    };
    //postion and pose robot
    double *robot_position;
    double *robot_position_get;
    geometry_msgs::PoseStamped pose_o,pose_n,pose_m;
    geometry_msgs::PoseStamped pose_o_robot,pose_n_robot,pose_m_robot; 
    // marker for robot
    class marker{
        public:
            data my_data;
            string marker_type;
            string marker_dir;
            string marker_data;
            void reset(int mode);
            // pose to save posstion
            double x_set=0;
            double y_set=0;
            double z_set=0;
            double w_set=1;
            vector<geometry_msgs::Pose> my_pose;
            vector<geometry_msgs::Pose> my_pose2;
            // detect
            int detect_vl();
            int detect_bar(float bar_distance);
            int detect_l();
            // offset transfrom
            double off_set_x=0.0;
            double off_set_y=0.0;
            double off_set_dis=0;
            double off_set_angle=0;
            int  caculate_transforms_ofset();
            // send tranfrom
            int check_send_transforms_tf_frame();
            int tranfrom_pose_marker(int mode);
            int check_first_tranfrom_pose_marker();
            int tranfrom_pose_marker2(int mode);
            // action maker
            int status=0;         // 0 nothing 1 collect & detect 2 active
            int active_step=0;
            int move_to_postion_pose_n();
            int move_to_orientation_pose_n();
            int action();
            // 
            int new_update=0;
            ros::Time t_send,t_get;
    };
    marker my_marker;
    // action robot
    int start;
    void process_data(string data);
    void robot_emg();
    void pub_cmd_vel(float v, float w);
    int get_footprint();
    int safe;
    float  x1_footprint,y1_footprint,x2_footprint,y2_footprint;
    float  safe_x1=0.01,safe_x2=0.01,safe_y1=0.01,safe_y2=0.01;
    sensor_msgs::LaserScan scan_safe;
    int check_safe();
    void pub_status_marker();
    void pub_status();
    #define mvibot_marker_define 1
#endif