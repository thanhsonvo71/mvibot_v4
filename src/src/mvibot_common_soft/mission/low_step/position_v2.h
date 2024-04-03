using namespace std;
// robot var
int tranfom_map_robot=0;
float *position_robot;
float position_goal[4];
int free_space_robot=1;
int check_free_space_robot=0;
int get_user_path=0;
int enable_ob1=-1;
int enable_ob2=-1;
//
long long movebase_goal_id=0;
long long exepath_goal_id=0; 
long long target_goal_id=0;
long long target_exepath_id=0;
// var for action mbf
mbf_msgs::MoveBaseGoal goal_pub;
nav_msgs::Path goal_path;
nav_msgs::Path user_path;
//
class position_{
	public:
        string data;
        string mode;
        int num_tab;
		float x;
		float y;
		float z;
		float w;
        int status=0;
        //
        int non_avoid=0;
        int stop_non_avoid=0;
        int count_ob=0;
        int count_non_ob=0;
        //
        void print(int n);
        void process_data();
        int action(int action);
        void reset(); 
};
void pub_request_map(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/request_map", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
            pub.publish(msg);
    } else {
        creat_fun=1;
        msg.data="1";
    }
}
void check_pose_costmap(){
    static ros::NodeHandle n;
    static ros::ServiceClient client = n.serviceClient<mbf_msgs::CheckPose>("/"+mvibot_seri+"/move_base_flex/check_pose_cost");
    static mbf_msgs::CheckPose srv;
    static float creat_fun=0;
    if(creat_fun==1)
    {
            srv.request.costmap=1;
            srv.request.safety_dist=0.0;
            srv.request.lethal_cost_mult=100.0;//=50.0;
            srv.request.inscrib_cost_mult=0;
            srv.request.unknown_cost_mult=0;
            srv.request.pose.header.stamp=ros::Time::now();
            srv.request.pose.pose.position.x=1.25; // 0.5 1.0
            srv.request.pose.pose.orientation.w=1.0;
            srv.request.pose.header.frame_id=mvibot_seri+"/base_footprint";
            srv.request.current_pose=false;
            // 
            static ros::Time t;
            t=ros::Time::now();
            pub_consolog("A|"+to_string((long double)t.sec+(long double)t.nsec*1e-9));
            client.call(srv);
            t=ros::Time::now();
            pub_consolog("B|"+to_string((long double)t.sec+(long double)t.nsec*1e-9));
            //
            if((srv.response.state)==0 | (srv.response.state)==1) free_space_robot=1;
            else free_space_robot=0; //1 0
            // cout<<free_space_robot<<endl;
            // cout<<"cost:"<<srv.response.cost<<endl;
            // cout<<"state:"<<to_string(srv.response.state)<<endl;
    } else creat_fun=1;
}
int check_pose_costmap_v2(double x, double y, double z, double w,double safe,string frame){
    static ros::NodeHandle n;
    static ros::ServiceClient client = n.serviceClient<mbf_msgs::CheckPose>("/"+mvibot_seri+"/move_base_flex/check_pose_cost");
    static mbf_msgs::CheckPose srv;
    static int creat_fun=0;
    if(creat_fun==1)
    {
            // param
            srv.request.costmap=1;
            srv.request.safety_dist=safe;
            srv.request.lethal_cost_mult=100.0;
            srv.request.inscrib_cost_mult=0;
            srv.request.unknown_cost_mult=0;
            srv.request.current_pose=false;
            //
            srv.request.pose.header.stamp=ros::Time::now();
            srv.request.pose.pose.position.x=x;
            srv.request.pose.pose.position.y=y;
            srv.request.pose.pose.position.z=0;
            srv.request.pose.pose.orientation.x=0.0;
            srv.request.pose.pose.orientation.y=0.0;
            srv.request.pose.pose.orientation.z=z;
            srv.request.pose.pose.orientation.w=w;
            srv.request.pose.header.frame_id=frame;
            //
            client.call(srv);
            if((srv.response.state)==0 | (srv.response.state)==1) return 1;
            else return 0;
    } else creat_fun=1;
    return 0;
}
void check_cost_path_v1(){
    if(check_free_space_robot==1){
        static int check_1,check_2,check_3;
        static float dis,dis2,dis3,dis4;
        static float x_path,y_path;
        static float x_path2,y_path2;
        static float x_path3,y_path3;
        static float x_path4,y_path4;
        static int error_AMCL;
        static int free1,free2,free3,free4;
        //
        free1=-1; free2=-1; free3=-1; free4=-1;
        check_1=0;
        check_2=0;
        check_3=0;
        error_AMCL=0;
        free_space_robot=0;
        // check 1st
        free1=check_pose_costmap_v2(0.75,0,0,1,0.02,mvibot_seri+"/base_footprint"); // 0.050.1
        //free1=1;
        if(free1==1){
            //
            for(int i=0;i<goal_path.poses.size();i++){
                x_path=goal_path.poses[i].pose.position.x;
                y_path=goal_path.poses[i].pose.position.y;
                dis=sqrt((position_robot[0]-x_path)*(position_robot[0]-x_path)+(position_robot[1]-y_path)*(position_robot[1]-y_path));
                if(dis<=0.2){
                    //start check path number 1
                    for(int j=i;j<goal_path.poses.size();j++){
                        x_path2=goal_path.poses[j].pose.position.x;
                        y_path2=goal_path.poses[j].pose.position.y;
                        dis2=sqrt((x_path2-x_path)*(x_path2-x_path)+(y_path2-y_path)*(y_path2-y_path));
                        if(dis2>=0.7){
                            check_1=1;
                            for(int k=j;k<goal_path.poses.size();k++){
                                x_path3=goal_path.poses[k].pose.position.x;
                                y_path3=goal_path.poses[k].pose.position.y;
                                dis3=sqrt((x_path3-x_path2)*(x_path3-x_path2)+(y_path3-y_path2)*(y_path3-y_path2));
                                if(dis3>=0.7){
                                    check_2=1;
                                    for(int m=k;m<goal_path.poses.size();m++){
                                        x_path4=goal_path.poses[m].pose.position.x;
                                        y_path4=goal_path.poses[m].pose.position.y;
                                        dis4=sqrt((x_path4-x_path3)*(x_path4-x_path3)+(y_path4-y_path3)*(y_path4-y_path3));
                                        if(dis4>=0.6){
                                            check_3=1;
                                            break;
                                        }
                                    }
                                    break;
                                }
                            }
                            break;
                        }
                    }
                    break;
                }
            }
            //
            if(error_AMCL==0){
                // 0.05 ->0.02
                static geometry_msgs::Quaternion quaternion;
                static tf2::Quaternion quaternion_tf2;
                if(check_1==1){
                    static float yaw1;
                    yaw1=atan2(y_path2-y_path,x_path2-x_path);
                    quaternion_tf2.setRPY(0, 0, yaw1);
                    quaternion=tf2::toMsg(quaternion_tf2);
                    free2=check_pose_costmap_v2(x_path2,y_path2,quaternion.z,quaternion.w,0.02,"map");
                    //send_tranfrom2(x_path2,y_path2,quaternion.z,quaternion.w,"map","ABC1");
                }
                if(check_2==1){
                    static float yaw2;
                    yaw2=atan2(y_path3-y_path2,x_path3-x_path2);
                    quaternion_tf2.setRPY(0, 0, yaw2);
                    quaternion=tf2::toMsg(quaternion_tf2);
                    free3=check_pose_costmap_v2(x_path3,y_path3,quaternion.z,quaternion.w,0.02,"map");
                    //send_tranfrom2(x_path3,y_path3,quaternion.z,quaternion.w,"map","ABC2");
                }
                if(check_3==1){
                    static float yaw3;
                    yaw3=atan2(y_path4-y_path3,x_path4-x_path3);
                    quaternion_tf2.setRPY(0, 0, yaw3);
                    quaternion=tf2::toMsg(quaternion_tf2);
                    free4=check_pose_costmap_v2(x_path4,y_path4,quaternion.z,quaternion.w,0.02,"map");
                    //send_tranfrom2(x_path4,y_path4,quaternion.z,quaternion.w,"map","ABC3");
                }
            }
        }
        //
        free_space_robot=1;
        if(free1==0) free_space_robot=0;
        if(free2==0) free_space_robot=0;
        if(free3==0) free_space_robot=0;
        if(free4==0) free_space_robot=0;
        pub_consolog(to_string(free1)+"|"+to_string(free2)+"|"+to_string(free3)+"|"+to_string(free4));
    }else{
        free_space_robot=1;
    }
}
float getyaw(float data3, float data4){
  	static geometry_msgs::Quaternion quat_msg;
	quat_msg.x=0;
	quat_msg.y=0;
	quat_msg.z=data3;
	quat_msg.w=data4;
	return tf::getYaw(quat_msg);
}
void pub_user_path(nav_msgs::Path user_path){
    	static ros::NodeHandle n;
        static ros::Publisher  pub = n.advertise<nav_msgs::Path>("/"+mvibot_seri+"/user_paths", 1);
        static float creat_fun=0;
        if(creat_fun==1)
        {
                pub.publish(user_path);
        } else creat_fun=1;
}
void collect_user_pathf(){
    if(get_user_path==1){
        if(user_path.poses.size()==0){
            user_path.poses.resize(1);
            user_path.poses[0].header.frame_id="map";
            user_path.poses[0].pose.position.x=position_robot[0];
            user_path.poses[0].pose.position.y=position_robot[1];
            user_path.poses[0].pose.orientation.z=position_robot[2];
            user_path.poses[0].pose.orientation.w=position_robot[3];
        }else{
            static float x1,x2,y1,y2;
            static int n;
            x1=position_robot[0];
            y1=position_robot[1];
            n=user_path.poses.size();
            x2=user_path.poses[n].pose.position.x;
            y2=user_path.poses[n].pose.position.y;
            if(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))>=0.05){
                user_path.poses.resize(n+1);
                user_path.poses[n].header.frame_id="map";
                user_path.poses[n].pose.position.x=position_robot[0];
                user_path.poses[n].pose.position.y=position_robot[1];
                user_path.poses[n].pose.orientation.z=position_robot[2];
                user_path.poses[n].pose.orientation.w=position_robot[3];
            }
        }
        pub_user_path(user_path);
    }else{
        if(get_user_path==0)
        if(user_path.poses.size()>0) user_path.poses.resize(0);
    }
}
// pub_cmd_vel
void pub_cmd_vel(float v,float w){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<geometry_msgs::Twist>("/"+mvibot_seri+"/cmd_vel", 1);
    static geometry_msgs::Twist cmd_msg;
    static float creat_fun=0;
	if(creat_fun==1)
	{
        cmd_msg.linear.x=(double)v;
        cmd_msg.angular.z=(double)w;
		//
        pub.publish(cmd_msg);
	} else creat_fun=1;
}
// define mbf
typedef actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>     GoalClient;  
typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction>      ExepathClient;
typedef actionlib::SimpleActionClient<mbf_msgs::GetPathAction>      GetpathClient;
typedef actionlib::SimpleActionClient<mbf_msgs::RecoveryAction>     RecoveryClient;
//  function action mbf
int action_goal(int mode);
void action_recovery(int mode);
int action_getpath(int mode);
int action_exepath(int mode);
// call clear costmap via service
void clear_costmap()
{
        static ros::NodeHandle n;
        static ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/"+mvibot_seri+"/move_base_flex/clear_costmaps");
        static std_srvs::Empty srv;
        static float creat_fun=0;
        if(creat_fun==1)
        {
                client.call(srv);
        } else creat_fun=1;
} 
//
int action_goal(int mode){
    static float creat_fun=0;
    static GoalClient action_goal("/"+mvibot_seri+"/move_base_flex/move_base", true);
    static mbf_msgs::MoveBaseGoal msg;
    static int send_cmd=0;
    if(creat_fun==0)
    {
        //
        while(!action_goal.waitForServer(ros::Duration(1.0))){
            ROS_INFO("Waiting for action_goal to come up");
            pub_request_map();
        }
        printf("Connection action goal server\n");
        creat_fun=1;
        //
        msg.target_pose.header.frame_id="map";
        msg.controller="DWAPlannerROS";
        return 0;
    }else {
        if(mode==0) {
            if(action_goal.getState().toString()=="ACTIVE") {
                action_goal.cancelAllGoals();
                action_goal.waitForResult();
                return Active_;
            }  
            else{
                if(action_goal.getState().toString()=="ABORTED"){
                    // action_goal.cancelAllGoals();
                    // action_goal.waitForResult();
                    msg.target_pose.pose.position.x=position_robot[0];
                    msg.target_pose.pose.position.y=position_robot[1];
                    msg.target_pose.pose.orientation.z=position_robot[2];
                    msg.target_pose.pose.orientation.w=position_robot[3];
                    //action_goal.sendGoal(msg);
                    //action_goal.waitForResult();
                    action_goal.cancelAllGoals();
                    return Finish_;
                }else{
                    action_goal.cancelAllGoals();
                    return Finish_;
                }
            }
        }
        else if(mode==1) {
            msg.target_pose.pose.position.x=position_goal[0];
            msg.target_pose.pose.position.y=position_goal[1];
            msg.target_pose.pose.orientation.z=position_goal[2];
            msg.target_pose.pose.orientation.w=position_goal[3];
            msg.target_pose.header.stamp=ros::Time::now();
            if(action_goal.getState().toString()!="ACTIVE"){
                action_goal.sendGoal(msg);
                return Finish_;
            }
            else{
                action_goal.cancelAllGoals();
                action_goal.waitForResult();
                return Active_;
            }
        }
        else if(mode==2){
            cout<<"\t \t \t \t \t "<<action_goal.getState().toString()<<"**************************"<<endl;
            if(action_goal.getState().toString()=="ACTIVE"){
                return Active_;
            }
            else if(action_goal.getState().toString()=="SUCCEEDED")    return Finish_;
            else return Error_;
        }
        else if(mode==3){
            // add from 1/2/2024
            msg.target_pose.pose.position.x=position_robot[0];
            msg.target_pose.pose.position.y=position_robot[1];
            msg.target_pose.pose.orientation.z=position_robot[2];
            msg.target_pose.pose.orientation.w=position_robot[3];
            msg.target_pose.header.stamp=ros::Time::now();
            action_goal.sendGoal(msg);
            action_goal.waitForResult();
            return Finish_;
        }
    }
    return 0;
}
void action_recovery(int mode){
    static float creat_fun=0;
    static RecoveryClient action_recovery("/"+mvibot_seri+"/move_base_flex/recovery", true);
    static mbf_msgs::RecoveryGoal msg;
    if(creat_fun==0)
    {
        //
        while(!action_recovery.waitForServer(ros::Duration(1.0))){
         ROS_INFO("Waiting for action_recovery to come up");
        }
        printf("Connection action_recovery server\n");
        creat_fun=1;
    }else{
        msg.behavior="clear_costmap_recovery";
        msg.concurrency_slot=10;
        action_recovery.sendGoal(msg);
        action_recovery.waitForResult();
        cout<<""<<action_recovery.getResult().get()->message<<endl;
        cout<<""<<action_recovery.getState().toString()<<endl;
    }
}
int action_getpath(int mode){
    static float creat_fun=0;
    static GetpathClient action_getpath("/"+mvibot_seri+"/move_base_flex/get_path", true);
    static mbf_msgs::GetPathGoal msg;
    if(creat_fun==0)
    {
        //
        while(!action_getpath.waitForServer(ros::Duration(1.0))){
         ROS_INFO("Waiting for action_getpath to come up");
        }
        printf("Connection action_getpath server\n");
        creat_fun=1;
        //
        msg.use_start_pose=false;
        msg.concurrency_slot=10;
        msg.tolerance=0.3;
        msg.target_pose.header.frame_id="map";
        //
        return 0;
    }else{
        msg.target_pose.pose.position.x=position_goal[0];
        msg.target_pose.pose.position.y=position_goal[1];
        msg.target_pose.pose.orientation.z=position_goal[2];
        msg.target_pose.pose.orientation.w=position_goal[3];
        action_getpath.sendGoal(msg);
        action_getpath.waitForResult();
        cout<<""<<action_getpath.getResult().get()->message<<endl;
        cout<<""<<action_getpath.getState().toString()<<endl; 
        pub_user_path(action_getpath.getResult().get()->path);
        goal_path=action_getpath.getResult().get()->path;
        if(action_getpath.getState().toString()=="SUCCEEDED") return Finish_;
        else return Error_;
    }

}
int action_exepath(int mode){
    static float creat_fun=0;
    static ExepathClient action_exepath("/"+mvibot_seri+"/move_base_flex/exe_path", true);
    static mbf_msgs::ExePathGoal msg;
    if(creat_fun==0)
    {
        //
        while(!action_exepath.waitForServer(ros::Duration(1.0))){
         ROS_INFO("Waiting for action_exepath to come up");
        }
        printf("Connection action_exepath server\n");
        creat_fun=1;
        msg.angle_tolerance=0.08;
        msg.dist_tolerance=0.35;
        msg.concurrency_slot=0;
        msg.tolerance_from_action=true;
        msg.controller="DWAPlannerROS";
        //
        return 0;
    }else{
        if(mode==0){
            msg.path.header.stamp=ros::Time::now();
            if(action_exepath.getState().toString()=="ACTIVE"){
                action_exepath.cancelAllGoals();
                action_exepath.waitForResult();
                return Active_;
            }else{
                if(action_exepath.getState().toString()=="ABORTED"){
                    action_exepath.cancelAllGoals();
                    action_exepath.waitForResult();
                    return Finish_;
                }else{
                    action_exepath.cancelAllGoals();
                    return Finish_;
                }
            }
        }
        else if(mode==1){
            msg.path=goal_path;
            //
            if(action_exepath.getState().toString()!="ACTIVE"){
                action_exepath.sendGoal(msg);
                return Finish_;
            }else{
                action_exepath.cancelAllGoals();
                action_exepath.waitForResult();
                return Active_;
            }
        }else if(mode==2){
            cout<<""<<action_exepath.getState().toString()<<endl;
            if(action_exepath.getState().toString()=="ACTIVE") return Active_;
            else if(action_exepath.getState().toString()=="SUCCEEDED")    return Finish_;
            else{   
                return Error_;
            }
        }
    }
    return 0;
}
//
void position_::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"~","=","~");
    //non_avoid=1;
    for(int i=0;i<data_return.data1.size();i++){
        if(data_return.data1[i]=="mode")          mode=data_return.data2[i];
        if(data_return.data1[i]=="x")             x=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="y")             y=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="z")             z=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="w")             w=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="non_avoid")     non_avoid=stoi_f(data_return.data2[i]);
    }
}
void position_::print(int n){
    num_tab=n;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"Data:"<<data<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"mode:"<<mode<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"x:"<<x<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"y:"<<y<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"z:"<<z<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"w:"<<w<<endl;
}
int position_::action(int action){
    //
    static int complete_position=0;
    static int is_error;
    static float x1=0,y1=0,z1=0,w1=0;
    static float x2=0,y2=0,z2=0,w2=0;
    static float sleep=0;
    //
    if(action==Active_){
        // check is robot aviable in goal
        x2=position_robot[0];
        y2=position_robot[1];
        z2=position_robot[2];
        w2=position_robot[3];
        //
        static float dis;
        dis=sqrt((x2-x)*(x2-x)+(y2-y)*(y2-y));
        static float angle1,angle2;
        angle2=getyaw(z2,w2);
        angle1=getyaw(z,w);
        //
        cout<<"theta:"<<fabs(sin(angle2)-sin(angle1))<<endl;
        cout<<"dis:"<<dis<<endl;
        //
        if(dis<=0.35){
            complete_position=1;
            if(fabs(sin(angle2)-sin(angle1))<=0.05 & fabs(cos(angle2)-cos(angle1))<=0.05) { //0.1 //0.08 only sin //0.05
                complete_position=2;
            }
        }else complete_position=0;
        //
        cout<<"mygoal: "<<position_goal[0]<<"|"<<position_goal[1]<<"|"<<position_goal[2]<<"|"<<position_goal[3]<<endl;
        if(mode=="normal"){
            if(status==0){
                cout<<"Step"<<status<<endl;
                // enalbe obstacle
                if(enable_ob1!=1)
                enable_ob1=stof_f(set_get_param("/"+mvibot_seri+"/move_base_flex/global_costmap/obstacles1/set_parameters","enabled","bool","1"));	
                if(enable_ob2!=1)
                enable_ob2=stof_f(set_get_param("/"+mvibot_seri+"/move_base_flex/global_costmap/obstacles2/set_parameters","enabled","bool","1"));
                // Check move base not active
                if(enable_ob1==1 & enable_ob2==1){
                    if(action_goal(0)==Finish_){
                        target_goal_id=movebase_goal_id;
                        status=1;
                    }
                }
                return Active_;
            }
            else if(status==1){
                cout<<"Step"<<status<<endl;
                // clear costmap
                clear_costmap();
                action_recovery(0);
                status=2;
                sleep=0;
                return Active_;
            }
            else if(status==2){
                cout<<"Step"<<status<<endl;
                // sleep 1s wait for move base get action goal
                sleep+=ts_mission_step_scan;
                if(sleep>=1.0){
                    sleep=0;
                    status=3;
                }
                return Active_;
            }
            else if(status==3){
                //send goal
                cout<<"Step"<<status<<endl;
                if(complete_position==1){
                    if(is_error){
                        position_goal[0]=position_robot[0];//+0.35;
                        position_goal[1]=position_robot[1];
                        position_goal[2]=z;
                        position_goal[3]=w;
                        is_error=0;
                    }else{
                        position_goal[0]=position_robot[0];
                        position_goal[1]=position_robot[1];
                        position_goal[2]=z;
                        position_goal[3]=w;
                    }
                }
                else if(complete_position==2){
                    position_goal[0]=position_robot[0];
                    position_goal[1]=position_robot[1];
                    position_goal[2]=position_robot[2];
                    position_goal[3]=position_robot[3];
                }
                else{
                    position_goal[0]=x;
                    position_goal[1]=y;
                    position_goal[2]=z;
                    position_goal[3]=w;
                }
                // because robot is in goal so finish
                if(complete_position==2){
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"finish goal"<<endl;
                    // reset variable
                    status=0;
                    my_path.poses.resize(0);
                    return Finish_;
                }
                else{
                    if(action_goal(1)==Finish_){
                        status=4;
                        sleep=0;
                    }
                }
                return Active_;
            }
            else if(status==4){
                cout<<"Step"<<status<<endl;
                // sleep 1s wait for move base get action goal
                sleep+=ts_mission_step_scan;
                if(sleep>=1.0){
                    sleep=0;
                    status=5;
                }
                return Active_;
            }
            else if(status==5){
                cout<<"Step"<<status<<endl;
                // check move base get goal ? 
                if(movebase_goal_id>target_goal_id){
                    //
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"Move base receive goal"<<endl;
                    status=6;
                }else{
                    //
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"Move base not receive action goal"<<endl;
                    status=0;
                }
                return Active_;
            }
            else if(status==6){
                cout<<"Step"<<status<<endl;
                static int get_action_goal_status;
                get_action_goal_status=action_goal(2);
                //
                if(get_action_goal_status==Finish_){
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"finish goal"<<endl;
                    // reset variable
                    status=0;
                    my_path.poses.resize(0);
                    return Finish_;
                }
                else if(get_action_goal_status==Active_){
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"mbf Active goal"<<endl;
                    return Active_;
                }
                else if(get_action_goal_status==Error_){
                    if(complete_position==2){
                        for(int j=0;j<num_tab;j++) cout<<"\t";
                        cout<<"finish goal"<<endl;
                        // reset variable
                        status=0;
                        my_path.poses.resize(0);
                        action_goal(3);
                        return Finish_;
                    }else{
                        for(int j=0;j<num_tab;j++) cout<<"\t";
                        cout<<"mbf error!"<<endl;
                        status=0;
                        is_error=1;
                        action_goal(3);
                        return Active_;
                    }
                }
            }
        }
        else if(mode=="line_follow"){
            cout<<status<<endl;
            if(status==0){
                // disable obstacle to find path
                if(enable_ob1!=0)
                enable_ob1=stof_f(set_get_param("/"+mvibot_seri+"/move_base_flex/global_costmap/obstacles1/set_parameters","enabled","bool","0"));	
                if(enable_ob2!=0)
                enable_ob2=stof_f(set_get_param("/"+mvibot_seri+"/move_base_flex/global_costmap/obstacles2/set_parameters","enabled","bool","0"));
                //
                if(enable_ob1==0 & enable_ob2==0){
                    if(action_exepath(0)==Finish_){
                        target_exepath_id=exepath_goal_id;
                        status=1;
                    }
                }
                //
                if(non_avoid==1) check_free_space_robot=1; 
                return Active_;
            }
            else if(status==1){
                clear_costmap();
                action_recovery(0);
                status=2;
                sleep=0;
                return Active_;
            }
            else if(status==2){
                cout<<"Step"<<status<<endl;
                // sleep 1s wait for move base get action goal
                sleep+=ts_mission_step_scan;
                if(sleep>=1.0){
                    sleep=0;
                    status=3;
                }
                return Active_;
            }
            else if(status==3){
                //check goal
                if(complete_position==1){
                    if(is_error){
                        position_goal[0]=position_robot[0]+0.35;
                        position_goal[1]=position_robot[1];
                        position_goal[2]=z;
                        position_goal[3]=w;
                        is_error=0;
                    }else{
                        position_goal[0]=position_robot[0];
                        position_goal[1]=position_robot[1];
                        position_goal[2]=z;
                        position_goal[3]=w;
                    }
                }
                else if(complete_position==2){
                    position_goal[0]=position_robot[0];
                    position_goal[1]=position_robot[1];
                    position_goal[2]=position_robot[2];
                    position_goal[3]=position_robot[3];
                }
                else{
                    position_goal[0]=x;
                    position_goal[1]=y;
                    position_goal[2]=z;
                    position_goal[3]=w;
                }
                // find path
                static int status_getpath;
                status_getpath=action_getpath(0);
                if(status_getpath!=Finish_)
                {
                    status=0;
                    return Error_;
                }else{
                    sleep=0;
                    status=4;
                }
                return Active_;
            }
            else if(status==4){
                if(complete_position==2){
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"finish path"<<endl;
                    // reset variable
                    status=0;
                    my_path.poses.resize(0);
                    goal_path.poses.resize(0);
                    check_free_space_robot=0;
                    count_non_ob=0;
                    count_ob=0;
                    return Finish_;
                }
                else{
                    // send goal
                    if(non_avoid==1){
                        sleep+=ts_mission_step_scan;
                        if(sleep<=4){
                            if(free_space_robot==1) count_non_ob++;
                            else count_non_ob=0;
                            //
                            if(count_non_ob>=60 | dis <= 1.5){
                                count_non_ob=0;
                                if(action_exepath(1)==Finish_){
                                    status=5;
                                    sleep=0;
                                }
                            }
                        }else{
                            // return step 0
                            sleep=0;
                            count_non_ob=0;
                            status=0;
                        }
                    }else{
                        if(action_exepath(1)==Finish_){
                            status=5;
                            sleep=0;
                        }
                    }
                    //
                }
                //
                return Active_;
            }
            else if(status==5){
                // sleep 1s wait for move base get action goal
                sleep+=ts_mission_step_scan;
                if(sleep>=1.0){
                    sleep=0;
                    status=6;
                }
                return Active_;               
            }
            else if(status==6){
                // check exepath_id 
                if(exepath_goal_id>target_goal_id){
                    //
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"Exe path receive goal"<<endl;
                    status=7;
                }else{
                    //
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"Exe path not receive action goal"<<endl;
                    status=0;
                }
                return Active_;
            }
            else if(status==7){
                static int get_action_goal_status;
                get_action_goal_status=action_exepath(2);
                if(get_action_goal_status==Finish_){
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"finish path"<<endl;
                    // reset variable
                    status=0;
                    my_path.poses.resize(0);
                    goal_path.poses.resize(0);
                    check_free_space_robot=0;
                    count_non_ob=0;
                    count_ob=0;
                    return Finish_;
                }
                else if(get_action_goal_status==Active_){
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"Exepath Active path"<<endl;
                    //
                    if(non_avoid==1){
                        if(free_space_robot==0 & dis>=1.5)
                        {
                            stop_non_avoid=1;
                            action_exepath(0);
                            pub_cmd_vel(0.0,0.0);
                            status=0;
                        }
                    }
                    return Active_;
                }
                else if(get_action_goal_status==Error_){
                    if(complete_position==2){
                        for(int j=0;j<num_tab;j++) cout<<"\t";
                        cout<<"finish path"<<endl;
                        // reset variable
                        status=0;
                        my_path.poses.resize(0);
                        goal_path.poses.resize(0);
                        check_free_space_robot=0;
                        return Finish_;
                    }else{
                        for(int j=0;j<num_tab;j++) cout<<"\t";
                        cout<<"mbf error!"<<endl;
                        status=0;
                        is_error=1;
                        return Active_;
                    }
                }
            }
        }
        return Active_;
   }else{
        my_path.poses.resize(0);
        goal_path.poses.resize(0);
        complete_position=0;
        status=0;
        sleep=0;
        is_error=0;
        //
        check_free_space_robot=0;
        count_ob=0;
        count_non_ob=0;
        if(mode=="normal") action_goal(0);
        if(mode=="line_follow") action_exepath(0);
        //
        position_goal[0]=-1;
        position_goal[1]=-1;
        position_goal[2]=-1;
        position_goal[3]=-1;
        return action;
   }
}
void position_::reset(){
    status=0;
    stop_non_avoid=0;
    count_ob=0;
    count_non_ob=0;
}