using namespace std;
// robot var
int tranfom_map_robot=0;
float *position_robot;
float position_goal[4];
int free_space_robot;
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
            client.call(srv);
            if((srv.response.state)==0 | (srv.response.state)==1) free_space_robot=1;
            else free_space_robot=1; //0
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
                action_goal.cancelGoal();
                action_goal.waitForResult();
                return Active_;
            }  
            else{
                if(action_goal.getState().toString()=="ABORTED"){
                    action_goal.cancelGoal();
                    action_goal.waitForResult();
                    // msg.target_pose.pose.position.x=position_robot[0];
                    // msg.target_pose.pose.position.y=position_robot[1];
                    // msg.target_pose.pose.orientation.z=position_robot[2];
                    // msg.target_pose.pose.orientation.w=position_robot[3];
                    // action_goal.sendGoal(msg);
                    // action_goal.waitForResult();            
                    // return Active_;
                    return Finish_;
                }else return Finish_;
            }
        }
        else if(mode==1) {
            msg.target_pose.pose.position.x=position_goal[0];
            msg.target_pose.pose.position.y=position_goal[1];
            msg.target_pose.pose.orientation.z=position_goal[2];
            msg.target_pose.pose.orientation.w=position_goal[3];
            if(action_goal.getState().toString()!="ACTIVE"){
                action_goal.sendGoal(msg);
                return Finish_;
            }
            else{
                action_goal.cancelGoal();
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
            if(action_exepath.getState().toString()=="ACTIVE"){
                action_exepath.cancelGoal();
                action_exepath.waitForResult();
                return Active_;
            }else{
                if(action_exepath.getState().toString()=="ABORTED"){
                    action_exepath.cancelGoal();
                    action_exepath.waitForResult();
                    return Finish_;
                }else return Finish_;
            }
        }
        else if(mode==1){
            msg.path=goal_path;
            //
            if(action_exepath.getState().toString()!="ACTIVE"){
                action_exepath.sendGoal(msg);
                return Finish_;
            }else{
                action_exepath.cancelGoal();
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
            if(fabs(sin(angle2)-sin(angle1))<=0.1 & fabs(cos(angle2)-cos(angle1))<=0.1) { //0.08 only sin
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
                action_recovery(0);
                status=2;
                return Active_;
            }
            else if(status==2){
                //send goal
                cout<<"Step"<<status<<endl;
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
                //
                if(action_goal(1)==Finish_){
                    status=3;
                    sleep=0;
                }
                return Active_;
            }
            else if(status==3){
                cout<<"Step"<<status<<endl;
                // sleep 1s wait for move base get action goal
                sleep+=ts_mission_step_scan;
                if(sleep>=1.0){
                    sleep=0;
                    status=4;
                }
                return Active_;
            }
            else if(status==4){
                cout<<"Step"<<status<<endl;
                // check move base get goal ? 
                if(movebase_goal_id>target_goal_id){
                    //
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"Move base receive goal"<<endl;
                    status=5;
                }else{
                    //
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"Move base not receive action goal"<<endl;
                    status=0;
                }
                return Active_;
            }
            else if(status==5){
                cout<<"Step"<<status<<endl;
                static int get_action_goal_status;
                get_action_goal_status=action_goal(2);
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
                return Active_;
            }
            else if(status==1){
                clear_costmap();
                action_recovery(0);
                status=2;
                return Active_;
            }
            else if(status==2){
                //send goal
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
                    my_path=goal_path;
                    status=0;
                    return Error_;
                }
                //
                if(action_exepath(1)==Finish_){
                    status=3;
                    sleep=0;
                }
                return Active_;
            }
            else if(status==3){
                // sleep 1s wait for move base get action goal
                sleep+=ts_mission_step_scan;
                if(sleep>=1.0){
                    sleep=0;
                    status=4;
                }
                return Active_;               
            }
            else if(status==4){
                // check exepath_id 
                if(exepath_goal_id>target_goal_id){
                    //
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"Exe path receive goal"<<endl;
                    status=5;
                }else{
                    //
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"Exe path not receive action goal"<<endl;
                    status=0;
                }
                return Active_;
            }
            else if(status==5){
                static int get_action_goal_status;
                get_action_goal_status=action_exepath(2);
                if(get_action_goal_status==Finish_){
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"finish path"<<endl;
                    // reset variable
                    status=0;
                    my_path.poses.resize(0);
                    return Finish_;
                }
                else if(get_action_goal_status==Active_){
                    for(int j=0;j<num_tab;j++) cout<<"\t";
                    cout<<"Exepath Active path"<<endl;
                    return Active_;
                }
                else if(get_action_goal_status==Error_){
                    if(complete_position==2){
                        for(int j=0;j<num_tab;j++) cout<<"\t";
                        cout<<"finish path"<<endl;
                        // reset variable
                        status=0;
                        my_path.poses.resize(0);
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
        complete_position=0;
        status=0;
        sleep=0;
        is_error=0;
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