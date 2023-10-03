using namespace std;
// robot var
int tranfom_map_robot=0;
float *position_robot;
float position_goal[4];
int free_space_robot;
int get_user_path=0;
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
            srv.request.pose.pose.position.x=0.5; // 0.5 1.0
            srv.request.pose.pose.orientation.w=1.0;
            srv.request.pose.header.frame_id=mvibot_seri+"/base_footprint";
            srv.request.current_pose=false;
            client.call(srv);
            if((srv.response.state)==0) free_space_robot=1;
            else free_space_robot=0;
            // cout<<free_space_robot<<endl;
            // cout<<"cost:"<<srv.response.cost<<endl;
            // cout<<"state:"<<to_string(srv.response.state)<<endl;
    } else creat_fun=1;
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
        return 0;
    }else {
        if(mode==0) {
            if(action_goal.getState().toString()=="ACTIVE") {
                action_goal.cancelGoal();
                action_goal.waitForResult();
            }  
            else{
                if(action_goal.getState().toString()=="ABORTED"){
                    msg.target_pose.pose.position.x=position_robot[0];
                    msg.target_pose.pose.position.y=position_robot[1];
                    msg.target_pose.pose.orientation.z=position_robot[2];
                    msg.target_pose.pose.orientation.w=position_robot[3];
                    action_goal.sendGoal(msg);
                    action_goal.waitForResult();
                }
            }
            send_cmd=0;   
            return 0;
        }
        else if(mode==1) {
            msg.target_pose.pose.position.x=position_goal[0];
            msg.target_pose.pose.position.y=position_goal[1];
            msg.target_pose.pose.orientation.z=position_goal[2];
            msg.target_pose.pose.orientation.w=position_goal[3];
            if(action_goal.getState().toString()!="ACTIVE"){
                //action_goal.
                if(action_goal.getState().toString()=="SUCCEEDED"){
                    static float dis_x,dis_y;
                    dis_x=position_goal[0]-position_robot[0];
                    dis_y=position_goal[1]-position_robot[1];
                    //
                    static float yaw1,yaw2;
                    yaw1=getyaw(position_goal[2],position_goal[3]);
                    yaw2=getyaw(position_robot[2],position_robot[3]);
                    if(sqrt(dis_x*dis_x+dis_y*dis_y)<=0.35 & (fabs(cos(yaw1)-cos(yaw2))< 0.1) & (fabs(sin(yaw1)-sin(yaw2))< 0.1))
                    {
                        return 1;
                    }else{
                        send_cmd++; 
                        if(send_cmd>=4){
                            action_goal.sendGoal(msg);
                            send_cmd=0;
                        }
                        return 0;
                    }
                }else{
                    send_cmd++;
                    if(send_cmd>=4){
                        action_goal.sendGoal(msg);
                        send_cmd=0;
                    }
                    return 0;
                }
            }
            else return 1;
        }
        else if(mode==2){
            cout<<"\t \t \t \t \t "<<action_goal.getState().toString()<<endl;
            if(action_goal.getState().toString()=="ACTIVE") return Active_;
            else if(action_goal.getState().toString()=="SUCCEEDED")    return Finish_;
            else  return Error_;
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
        msg.concurrency_slot=1;
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
        msg.concurrency_slot=1;
        msg.tolerance=0.2;
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
        msg.dist_tolerance=0.2;
        msg.concurrency_slot=0;
        msg.tolerance_from_action=true;
        msg.controller="DWAPlannerROS";
        //
        return 0;
    }else{
       
        if(mode==0 & action_exepath.getState().toString()=="ACTIVE") action_exepath.cancelGoal();
        if(mode==1){
             msg.path=goal_path;
             action_exepath.sendGoal(msg);
        }
        else if(mode==2){
            cout<<""<<action_exepath.getState().toString()<<endl;
            if(action_exepath.getState().toString()=="ACTIVE") return Active_;
            else if(action_exepath.getState().toString()=="SUCCEEDED")    return Finish_;
            else  return Error_;
        }
        return Active_;
    }
}
//
void position_::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"~","=","~");
    for(int i=0;i<data_return.data1.size();i++){
        if(data_return.data1[i]=="mode")  mode=data_return.data2[i];
        if(data_return.data1[i]=="x")     x=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="y")     y=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="z")     z=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="w")     w=stof_f(data_return.data2[i]);
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
    if(action==Active_){
        if(mode=="normal"){
            if(status==0){
                for(int j=0;j<num_tab;j++) cout<<"\t";
                cout<<"clear costmap"<<endl;
                action_recovery(0);
                status=1;
            }else if(status==1){
                for(int j=0;j<num_tab;j++) cout<<"\t";
                cout<<"send goal"<<endl;
                position_goal[0]=x;
                position_goal[1]=y;
                position_goal[2]=z;
                position_goal[3]=w;
                static int res;
                res=action_goal(1);
                if(res==1) status=2;
            }else if(status==2){
                static int get_action_goal_status;
                static int n;
                get_action_goal_status=action_goal(2);
                if(get_action_goal_status==Active_){
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"mbf Active goal"<<endl;
                }
                if(get_action_goal_status==Error_) {
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"replay goal because mbf error"<<endl;
                    status=0;
                }
                if(get_action_goal_status==Finish_){
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"Check finish goal"<<endl;
                    //
                    static float dis_x,dis_y;
                    dis_x=x-position_robot[0];
                    dis_y=y-position_robot[1];
                    //
                    static float yaw1,yaw2;
                    yaw1=getyaw(z,w);
                    yaw2=getyaw(position_robot[2],position_robot[3]);
                    if(sqrt(dis_x*dis_x+dis_y*dis_y)<=0.35 & (fabs(cos(yaw1)-cos(yaw2))< 0.1) & (fabs(sin(yaw1)-sin(yaw2))< 0.1)){
                        for(int j=0;j<num_tab;j++) cout<<"\t";
                        cout<<"finish goal"<<endl;
                        status=0;
                        return Finish_;
                    }else{
                        for(int j=0;j<num_tab;j++) cout<<"\t";
                        cout<<"replay goal because this goal is corret with set goal"<<endl;
                        status=0;
                    }   
                }
            }
        }else if(mode=="line_follow"){
            static float dis_x,dis_y,error_dis;
            dis_x=x-position_robot[0];
            dis_y=y-position_robot[1];
            error_dis=sqrt(dis_x*dis_x+dis_y*dis_y);
            if(status==0){
                for(int j=0;j<num_tab;j++) cout<<"\t";
                cout<<"clear costmap"<<endl;
                action_recovery(0);
                status=1;
            }else if(status==1){
                static int status_getpath;
                for(int j=0;j<num_tab;j++) cout<<"\t";
                cout<<"caculate path"<<endl;
                position_goal[0]=x;
                position_goal[1]=y;
                position_goal[2]=z;
                position_goal[3]=w;
                status_getpath=action_getpath(0);
                if(status_getpath==Finish_) status=2;
                else {
                    status=0;
                    return Error_;
                }
            }else if(status==2){
                static int n_free_space_robot;
                if(free_space_robot==1 | error_dis <= 1.0){
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"send path"<<endl;
                    action_exepath(1);
                    status=3;
                    n_free_space_robot=0;
                }else{
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"no free space to ative"<<endl;
                    action_exepath(0);
                    n_free_space_robot++;
                }
            }else if(status==3){
                static int status_exepath;
                if(free_space_robot==0 & error_dis > 1.0 ) {
                    status_exepath=action_exepath(0);
                    status=2;
                }else{
                    status_exepath=action_exepath(2);
                    if(status_exepath==Finish_){
                        status=0;
                        return Finish_;
                    }else if(status_exepath==Active_) return Active_;
                    else if(status_exepath==Error_){
                        status=2;
                    } 
                }
                
            }
        }
        return Active_;
   }else{
        status=0;
        action_goal(0);
        action_exepath(0);
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
}