//
#include "src/common/libary/libary_basic.h"
#include "src/common/libary/libary_ros.h"
#include "src/common/hardware/hardware.h"
#include "src/common/thread/thread.h"
//
#include "src/mvibot_common_soft/mvibot_common_soft_init.h"
#include "src/mvibot_common_soft/mission/mission_init.h"
//
#include "src/common/get_position/get_position.h"
#include "src/common/set_get_param/set_get_param.h"
#include "src/common/read_file/read_file.h"
#include "src/common/history/history.h"
using namespace std;
// var for action
multiple_mission my_multiple_mission;
mission my_mission_charge_battery;
mission my_mission_error;
int motor_right_ready=0,motor_left_ready=0;
float battery_soc_set_mission=-1;
float battery_soc=-1;
float battery_soc1=-1;
float battery_soc2=-1;
float want_to_charge=0;
int enable_ecoder_local_costmap=-1;
int enable_ecoder_global_costmap=-1;
float v_backward=-1;
float switch_for_hook=0;
// settime process
long double ts_process1=1.0; //time set for process1
long double ts_process2=0.1; //time set for process2
long double ts_process3=0.05; //time set for process3
static pthread_mutex_t process_mutex=PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
static pthread_t p_process1;
static pthread_t p_process2;
static pthread_t p_process3;
// create function system
void lock();
void unlock();
void function1();
void function2();
void function3();
void time_now(string data);
// creat thread
void * process1(void * nothing){
    char name[]="Process A";
	  process(name,ts_process1,0,function1);
    void *value_return;
    return value_return;
}
void * process2(void * nothing){
	  char name[]="Process B";
	  process(name,ts_process2,1,function2);
    void *value_return;
    return value_return;
}
void * process3(void * nothing){
	char name[]="Process C";
	process(name,ts_process3,2,function3);
    void *value_return;
    return value_return;
}
void pub_sound(float user_sound);
void save_mission_class(string data,string name);
void set_amcl(float x,float y,float z,float w);
void set_sound();
void set_led();
void pub_led(float red, float green, float blue, float ll, float lr, float lb, float lf);
//
void pub_led(float red, float green, float blue, float ll, float lr, float lb, float lf){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::Float32MultiArray>("/"+mvibot_seri+"/set_led", 1);
    static float creat_fun=0;
    static std_msgs::Float32MultiArray msg;
    if(creat_fun==1)
    {
            msg.data.resize(7);
            msg.data[0]=red/100*255;
            msg.data[1]=green/100*255;
            msg.data[2]=blue/100*255;
            msg.data[3]=ll;
            msg.data[4]=lr;
            msg.data[5]=lb;
            msg.data[6]=lf;
            pub.publish(msg);
    } else creat_fun=1;
}
void set_led(){
    if(tranfom_map_robot==0) pub_led(100,100,100,1,1,1,1);
    else {
        if(action_mission==Active_) {
            if(action_mode_mission==Mission_normal_){
                if(type_action_step=="sleep") pub_led(100,0,100,3,3,3,1);
                else if(type_action_step=="gpio" | type_action_step=="gpio_module") pub_led(100,0,100,2,2,2,1);
                else if(type_action_step=="marker")  pub_led(100,0,100,1,1,1,1);
                else pub_led(0,100,0,1,1,1,1);
            }else if(action_mode_mission==Mission_charge_battery_){
                pub_led(0,100,100,2,2,2,1);
            }
        }
        else{
             if(action_mission==Error_) pub_led(100,0,0,1,1,1,1);
             else {
                if(action_mode_mission==Mission_normal_){
                    if(my_multiple_mission.multiple_mission.size()==0)  pub_led(100,60,0,2,2,2,1);
                    else{
                        if(my_multiple_mission.num_mission_action!=-1)  pub_led(100,60,0,1,1,1,1);
                        else pub_led(100,60,0,3,3,3,1);
                    }
                }
                else pub_led(0,100,100,1,1,1,1);
             }
        }
    }
}
void set_sound(){
    static int sound_f;
    if(action_mission==Active_){
        pub_sound(sound);
        sound_f=1;
    }else{
        if(action_mission!=Finish_){
            if(action_mission==Error_){
                //pub_sound(sound);
                pub_sound(sound2);// ?
                sound_f=1;
            }
            else {
                if(sound_f==1) {
                    pub_sound(0);
                    sound_f=0;
                }
            }
        } else {
            if(sound_f==1) {
                pub_sound(0);
                sound_f=0;
            }
        }
    }
}
void pub_sound(float user_sound){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::Float32>("/"+mvibot_seri+"/music_start", 1);
    static float creat_fun=0;
    static std_msgs::Float32 msg;
    if(creat_fun==1)
    {
        msg.data=user_sound;
        pub.publish(msg);
    } else creat_fun=1;
}
void save_mission_class(string data,string name){
    static string cmd;
    cmd="";
    cmd=cmd+"echo"+" '"+data+"'"+" > "+define_path+"param/"+name+".yaml";
    system(cmd.c_str());
}
void set_amcl(float x,float y,float z,float w){
	static ros::NodeHandle n;
        static ros::Publisher  pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/"+mvibot_seri+"/initialpose", 1);
        static geometry_msgs::PoseWithCovarianceStamped pose;
        static float creat_fun=0;
	    static float cov[36];
        if(creat_fun==1)
        {
                pose.header.frame_id="map";
                pose.pose.pose.position.x=x;
                pose.pose.pose.position.y=y;
                pose.pose.pose.orientation.z=z;
                pose.pose.pose.orientation.w=w;
                pose.pose.covariance[0]=0.25;
                pose.pose.covariance[7]=0.25;
                pose.pose.covariance[35]=0.06853892326654787;
                pub.publish(pose);
        } else creat_fun=1;
}
// function recevice msg from ros
void mission_normalf(const std_msgs::String& msg){
	lock();
        if(action_mission!=Active_){
            static string data;
            data="";
            for(int i=0;i<msg.data.length();i++){
                if(msg.data[i]!=' ' & msg.data[i]!='\n' & msg.data!="\t") data+=msg.data[i];
            }
            //
            my_multiple_mission.reset();
            my_multiple_mission.delete_free();
            my_multiple_mission.data=data;
            my_multiple_mission.process_data();
            my_multiple_mission.print(0);
            save_mission_class(data,"mission");
            if(my_multiple_mission.get_list_mission_name()=="") send_history("warning","Robot reset mission normal");
            else send_history("normal","Robot recevice multiple mission normal: "+my_multiple_mission.get_list_mission_name());
        }
	unlock();
}
void mission_errorf(const std_msgs::String& msg){
	lock();
        if(action_mission!=Active_){
            static string data;
            data="";
            for(int i=0;i<msg.data.length();i++){
                if(msg.data[i]!=' ' & msg.data[i]!='\n' & msg.data!="\t") data+=msg.data[i];
            }
            //
            my_mission_error.reset();
            my_mission_error.delete_free();
            my_mission_error.data=data;
            my_mission_error.process_data();
            my_mission_error.print(0);
            save_mission_class(data,"mission_error");
            if(my_mission_error.name_mission=="") send_history("warning","Robot reset mission error");
            else send_history("normal","Robot recevice mission error: "+my_mission_error.name_mission);
        }
	unlock();
}
void mission_charge_batteryf(const std_msgs::String& msg){
	lock();
        if(action_mission!=Active_){
            static string data;
            data="";
            for(int i=0;i<msg.data.length();i++){
                if(msg.data[i]!=' ' & msg.data[i]!='\n' & msg.data!="\t") data+=msg.data[i];
            }
            //
            my_mission_charge_battery.reset();
            my_mission_charge_battery.delete_free();
            my_mission_charge_battery.data=data;
            my_mission_charge_battery.process_data();
            my_mission_charge_battery.print(0);
            save_mission_class(data,"mission_charge_battery");
            //
            if(my_mission_charge_battery.name_mission=="") send_history("warning","Robot reset mission charge battery");
            else send_history("normal","Robot recevice mission charge battery: "+my_mission_charge_battery.name_mission);
            //
        }
	unlock();
}
void mission_actionf(const std_msgs::String& msg){
	lock();
        if(msg.data=="0"){
           if(action_mission==Active_) send_history("normal","Stop action mission via web");
           if(action_mission==Error_)  send_history("normal","Reset error mission via web");
           action_mission=Cancel_;
        }
        else if(msg.data=="1")
        {
            send_history("normal","Try continue mission via web");
            action_mission=Active_;
        }
        else if(msg.data=="2")
        action_mission=Skip_;
	unlock();
}
void get_user_pathf(const std_msgs::String& msg){
	lock();
		// if(msg.data=="0") get_user_path=0;
        //     else if(msg.data=="1") get_user_path=1;
        //     else get_user_path=2;
	unlock();
}
void input_user_statusf(const std_msgs::Float32MultiArray& msg){
    lock();
    	input_user_status=msg;
    unlock();
}
void output_user_statusf(const std_msgs::Float32MultiArray& msg){
    lock();
    	output_user_status=msg;
    unlock();
}
void input_user_status_stringf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        //
        static std_msgs::Float32MultiArray input;
        input.data.resize(0);
        input.data.resize(data.data1.size()-1);
        for(int j=1;j<data.data1.size();j++){
            static string_Iv2 data2;
            data2.detect(data.data1[j],"",":","");
            input.data[j-1]=stof(data2.data1[1]);
        }
        //
        static int is_have;
        is_have=0;
        for(int i=0;i<my_module.size();i++){
            if(data.data1[0]==my_module[i].name){
                is_have=1;
                my_module[i].input_user=input;
                break;
            }
        }
        //
        if(is_have==0){
            cout<<"Have_new_moudle:"<<data.data1[0]<<endl;
            send_history("normal","Find new module "+data.data1[0]);
            my_module.resize(my_module.size()+1);
            //
            my_module[my_module.size()-1].name=data.data1[0];
            //
            my_module[my_module.size()-1].input_user=input;
        }   
    unlock();
}
void output_user_status_stringf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        //
        static std_msgs::Float32MultiArray output;
        output.data.resize(0);
        output.data.resize(data.data1.size()-1);
        for(int j=1;j<data.data1.size();j++){
            static string_Iv2 data2;
            data2.detect(data.data1[j],"",":","");
            output.data[j-1]=stof(data2.data1[1]);
        }
        //
        static int is_have;
        is_have=0;
        for(int i=0;i<my_module.size();i++){
            if(data.data1[0]==my_module[i].name){
                is_have=1;
                my_module[i].output_user=output;
                break;
            }
        }
        //
        if(is_have==0){
            cout<<"Have_new_moudle:"<<data.data1[0]<<endl;
            send_history("normal","Find new module "+data.data1[0]);
            my_module.resize(my_module.size()+1);
            //
            my_module[my_module.size()-1].name=data.data1[0];
            //
            my_module[my_module.size()-1].output_user=output;
        }
    unlock();
}
void battery_statusf(const std_msgs::String& msg){
    lock();
        static string_Iv2 data;
        data.detect(msg.data,"","|","");
        for(int i=1;i<data.data1.size();i++){
            static string_Iv2 data2;
            data2.detect(data.data1[i],"",":","");
            if(data2.data1[0]=="soc"){
                battery_soc=stof_f(data2.data1[1]);
            }
        }
    unlock();
}
void mission_mode_resetf(const std_msgs::String& msg){
    lock();
    // if(action_mission!=Active_ & action_mission!=Error_){
    //     if(msg.data=="1"){
    //         action_mode_mission=Mission_normal_;
    //         my_multiple_mission_charge_battery.action_step_II=0;
    //     }else if(msg.data=="2"){
    //         want_to_charge=1;
    //     }
    // }   
    unlock();
}
void hook_switchf(const std_msgs::Float32 & msg){
    lock();
        // swicth_for_hook=msg.data;
    unlock();
}
void status_markerf(const std_msgs::String & msg){
    lock();
        if(msg.data=="0") status_action_marker=0;
        if(msg.data=="1") status_action_marker=1;
        if(msg.data=="2") status_action_marker=2;
    unlock();
}
void initialpose_web(const std_msgs::String &msg)
{
	lock();
		static float data[4];
		static string string_data;
		static int j=0;
		j=0;
		string_data="";
		for(int i=0;i<msg.data.length();i++){
			if(msg.data[i]=='(' | msg.data[i]=='|'){
				string_data="";
				for(int k=i+1;k<msg.data.length();k++){
					if(msg.data[k]==')' | msg.data[k]=='|'){
						i=k-1;
					 	break;
					}
					else{
						string_data=string_data+msg.data[k];
					}
				}
				data[j]=stof(string_data);
				j++;
			}	
		}
		set_amcl(data[0],data[1],data[2],data[3]);
        send_history("normal","Set position for robot");
	unlock();
}
void motor_right_statusf(const std_msgs::String &msg)
{
	lock();
		static string_Iv2 data;
        static int live=-1,live_f=-1,brake=-1,brake_f=-1,enable=-1,enable_f=-1,error=-1,error_f=-1;
        static int is_ready;
        is_ready=1;
        data.detect(msg.data,"","|","");
        for(int i=0;i<data.data1.size();i++){
                static string_Iv2 data2;
                data2.detect(data.data1[i],"",":","");
                if(data2.data1.size()==2){
                    // if(data2.data1[0]=="live" & data2.data1[1]== "0")       is_ready=0;
                    // if(data2.data1[0]=="enable" & data2.data1[1]== "0")     is_ready=0;
                    // if(data2.data1[0]=="brake" & data2.data1[1]== "0")      is_ready=0;
                    if(data2.data1[0]=="live")      live=stoi_f(data2.data1[1]);
                    if(data2.data1[0]=="enable")    enable=stoi_f(data2.data1[1]);
                    if(data2.data1[0]=="brake")     brake=stoi_f(data2.data1[1]);
                    if(data2.data1[0]=="error")     error=stoi_f(data2.data1[1]);
                }
            }
            //
            if(live==0) is_ready=0;
            else{
                if(enable==0) is_ready=0;
                if(brake==0)  is_ready=0;
            }
            motor_right_ready=is_ready;
            //
	unlock();
}
void motor_left_statusf(const std_msgs::String &msg)
{
	lock();
		static string_Iv2 data;
        static int is_ready;
        static int live=-1,live_f=-1,brake=-1,brake_f=-1,enable=-1,enable_f=-1,error=-1,error_f=-1;
        is_ready=1;
        data.detect(msg.data,"","|","");
        for(int i=0;i<data.data1.size();i++){
            static string_Iv2 data2;
            data2.detect(data.data1[i],"",":","");
            if(data2.data1.size()==2){
                if(data2.data1[0]=="live")      live=stoi_f(data2.data1[1]);
                if(data2.data1[0]=="enable")    enable=stoi_f(data2.data1[1]);
                if(data2.data1[0]=="brake")     brake=stoi_f(data2.data1[1]);
                if(data2.data1[0]=="error")     error=stoi_f(data2.data1[1]);
            }
        }
        //
        if(live==0) is_ready=0;
        else{
            if(enable==0) is_ready=0;
            if(brake==0)  is_ready=0;
        }
        motor_left_ready=is_ready;
    unlock();
}
void move_base_statusf(const actionlib_msgs::GoalStatusArray & msg){
    lock();
        static string_Iv2 data;
        if(msg.status_list.size()>0){
            data.detect(msg.status_list[msg.status_list.size()-1].goal_id.id,"","-","");
            movebase_goal_id=stoll(data.data1[1]);
        }
    unlock();
}
void exe_path_statusf(const actionlib_msgs::GoalStatusArray & msg){
    lock();
        static string_Iv2 data;
        if(msg.status_list.size()>0){
            data.detect(msg.status_list[msg.status_list.size()-1].goal_id.id,"","-","");
            exepath_goal_id=stoll(data.data1[1]);
        }
    unlock();
}
void planf(const nav_msgs::Path & msg){
    lock();
        my_path=msg;
    unlock();
}
//
void pub_mission_action_infor(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/mission_action_infor", 10);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
        msg.data="";
        //
        msg.data=msg.data+"/Status>";
        if(action_mission==N_A_) msg.data=msg.data+"NA/";
        if(action_mission==Finish_) msg.data=msg.data+"Finish/";
        if(action_mission==Active_) msg.data=msg.data+"Active/";
        if(action_mission==Cancel_) msg.data=msg.data+"Cancel/";
        if(action_mission==Error_) msg.data=msg.data+"Error/";
        if(action_mission==Stop_) msg.data=msg.data+"Stop/";
        //
        msg.data=msg.data+"/mission_action_infor>";
        if(action_mission==Error_) {
            msg.data=msg.data+"Error/";
            msg.data=msg.data+"/Error_mission_infor:"+my_mission_error.get_infor(2)+"/";
        }
        else{
            if(action_mode_mission==Mission_normal_){
                msg.data=msg.data+"Normal/";
                msg.data=msg.data+"/Normal_mission_infor:"+my_multiple_mission.get_infor(0)+"/";
            }
            if(action_mode_mission==Mission_charge_battery_){
                msg.data=msg.data+"Battery/";
                msg.data=msg.data+"/Battery_mission_infor:"+my_mission_charge_battery.get_infor(1)+"/";
            }
        }
        //
        pub.publish(msg);
    } else creat_fun=1;
}
void pub_local_variable(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/local_variable", 10);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
        msg.data="";
        for(int i=0;i<my_vars_local.var.size();i++){
            msg.data=msg.data+"("+my_vars_local.var[i].name+":"+to_string(my_vars_local.var[i].data)+")";
        }
        pub.publish(msg);
    } else creat_fun=1;
}
void pub_mission_memory(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/mission_memory", 10);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
        msg.data="";
        //
        msg.data=msg.data+"/Normal_mission:";
        msg.data=msg.data+my_multiple_mission.get_list_mission_name();
        msg.data=msg.data+"/";
        //
        msg.data=msg.data+"/Battery_mission:";
        msg.data=msg.data+my_mission_charge_battery.name_mission;
        msg.data=msg.data+"/";
        //
        msg.data=msg.data+"/Error_mission:";
        msg.data=msg.data+my_mission_error.name_mission;
        msg.data=msg.data+"/";
        //
        pub.publish(msg);
    } else creat_fun=1;
}
void pub_footprint_infor(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/footprint", 10);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
        msg.data="";
        msg.data=to_string(footprint_x1)+"|"+to_string(footprint_x2)+"|"+to_string(footprint_y1)+"|"+to_string(footprint_y2);
        //
        pub.publish(msg);
    } else creat_fun=1;
}
void pub_path_infor(){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/path", 10);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
        //
        static float dis;
        static float x1,y1,x2,y2;
        //
        msg.data="";
        if(my_path.poses.size()>0){
            x1=my_path.poses[0].pose.position.x;
            y1=my_path.poses[0].pose.position.y;
            //
            msg.data=to_string(x1);
            msg.data+="|"+to_string(y1);
        }
        for(int i=0;i<my_path.poses.size();i++){
            x2=my_path.poses[i].pose.position.x;
            y2=my_path.poses[i].pose.position.y;
            dis=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
            if(dis>=0.5 | i == (my_path.poses.size()-1))
            {
                msg.data+="|"+to_string(x2);
                msg.data+="|"+to_string(y2);
                x1=x2;
                y1=y2;
            }
        }
        //
        pub.publish(msg);
    } else creat_fun=1;
}
//
std::string load_file(string name_file){
    //
    static string value_return;
    try
    {
	    std::ifstream file(define_path+"config/"+name_file);
	    std::string str; 
	    std::string data;
        std::getline(file, str);
        value_return=str;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        value_return="-1";
    }
    return value_return;
}
int  main(int argc, char** argv){
    ros::init(argc, argv, "mvibot_conmmon_soft_v2");
    ros::NodeHandle nh("~");
    //
    nh.getParam("mvibot_seri", mvibot_seri);
    static string mode;
    nh.getParam("mode", mode);
    while(mode!="navigation"){
       sleep(1);
    }
    //
    ts_mission_step_scan=(float)ts_process3;
    user_path.header.frame_id="map";
    //
    pub_history("");
    //
    action_goal(0);
    action_recovery(0);
    action_getpath(0);
    action_exepath(0);
    // load battery soc set for charge 
    battery_soc_set_mission=stof_f(load_file("robot_low_battery"));
    // load file mission
    ifstream file;
    string str,data;
    file.open(define_path+"param/mission.yaml");
        data="";
	    while (std::getline(file, str))
	    {
            // Process str
            data=data+str;
	    }
        my_multiple_mission.delete_free();
        my_multiple_mission.data=data;
        my_multiple_mission.process_data();
        my_multiple_mission.print(0);
        //
        if(my_multiple_mission.get_list_mission_name()=="") send_history("warning","Robot load mission normal: empty");
        else send_history("normal","Robot load mission normal: "+my_multiple_mission.get_list_mission_name());
        //
    file.close();
    file.open(define_path+"param/mission_error.yaml");
        data="";
	    while (std::getline(file, str))
	    {
            // Process str
            data=data+str;
	    }
        my_mission_error.delete_free();
        my_mission_error.data=data;        
        my_mission_error.process_data();
        my_mission_error.print(0);
        //
        if(my_mission_error.name_mission=="") send_history("warning","Robot load mission error: empty");
        else send_history("warning","Robot load mission error: "+my_mission_error.name_mission);
        //
    file.close();
    file.open(define_path+"param/mission_charge_battery.yaml");
        data="";
	    while (std::getline(file, str))
	    {
            // Process str
            data=data+str;
	    }
        my_mission_charge_battery.delete_free();
        my_mission_charge_battery.data=data;
        my_mission_charge_battery.process_data();
        my_mission_charge_battery.print(0);
        //
        if(my_mission_charge_battery.name_mission=="") send_history("warning","Robot load mission charge battery: empty");
        else send_history("warning","Robot load mission charge battery: "+my_mission_charge_battery.name_mission);
        //
    file.close();
    //
    pub_sound(0);
    pub_user_path(user_path);
    pub_led(0,0,0,0,0,0,0);
    set_amcl(0,0,0,1);
    //
    pub_gpio_set("");
    pub_gpio_msg_common("");
    //
    static int res;
    res=pthread_create(&p_process1,NULL,process1,NULL);
    res=pthread_create(&p_process2,NULL,process2,NULL); 
    res=pthread_create(&p_process3,NULL,process3,NULL);
    //
    ros::NodeHandle n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n14,n15,n16,n17,n18,n19,n20,n21;
    //ros::Subscriber sub1 = n1.subscribe("/"+mvibot_seri+"/msg", 1, msgf);   
    ros::Subscriber sub2 = n2.subscribe("/"+mvibot_seri+"/mission_normal", 1, mission_normalf); 
    ros::Subscriber sub3 = n3.subscribe("/"+mvibot_seri+"/mission_action", 1, mission_actionf); 
    ros::Subscriber sub4 = n4.subscribe("/"+mvibot_seri+"/get_user_path", 1, get_user_pathf);
    ros::Subscriber sub5 = n5.subscribe("/"+mvibot_seri+"/input_user_status", 1, input_user_statusf); 
    ros::Subscriber sub6 = n6.subscribe("/"+mvibot_seri+"/hook_switch", 1, hook_switchf); 
    ros::Subscriber sub7 = n7.subscribe("/"+mvibot_seri+"/status_marker", 1, status_markerf); 
    ros::Subscriber sub8 = n8.subscribe("/"+mvibot_seri+"/initialpose_web", 1, initialpose_web);
    ros::Subscriber sub9 = n9.subscribe("/"+mvibot_seri+"/output_user_status", 1, output_user_statusf); 
    // // I/O comon
    ros::Subscriber sub10 = n10.subscribe("/input_user_status_string", 100, input_user_status_stringf);
    ros::Subscriber sub11 = n11.subscribe("/output_user_status_string", 100, output_user_status_stringf);
    // // error mission
    ros::Subscriber sub12 = n12.subscribe("/"+mvibot_seri+"/mission_error", 1, mission_errorf);
    // // battery mission 
    ros::Subscriber sub14 = n14.subscribe("/"+mvibot_seri+"/mission_charge_battery", 1, mission_charge_batteryf);
    ros::Subscriber sub15 = n15.subscribe("/"+mvibot_seri+"/battery_status", 1, battery_statusf);
    ros::Subscriber sub16 = n16.subscribe("/"+mvibot_seri+"/mission_mode_reset", 1, mission_mode_resetf);
    //
    ros::Subscriber sub17 = n17.subscribe("/"+mvibot_seri+"/motor_right_status", 1, motor_right_statusf);
    ros::Subscriber sub18 = n18.subscribe("/"+mvibot_seri+"/motor_left_status", 1, motor_left_statusf);
    ros::Subscriber sub19 = n19.subscribe("/"+mvibot_seri+"/move_base_flex/move_base/status", 1, move_base_statusf);
    ros::Subscriber sub20 = n20.subscribe("/"+mvibot_seri+"/move_base_flex/exe_path/status", 1, exe_path_statusf);
    ros::Subscriber sub21 = n21.subscribe("/"+mvibot_seri+"/move_base_flex/NavfnROS/plan", 1, planf);
    ros::spin(); 
    return 0; 
}
void lock(){
    pthread_mutex_lock(&process_mutex);
}
void unlock(){
    pthread_mutex_unlock(&process_mutex);
}
void time_now(string name){
    lock();
        static struct timespec realtime;
        clock_gettime(CLOCK_REALTIME, &realtime);
        cout<<name+"|Time:"<<std::fixed << std::setprecision(5)<<((long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9)<<endl;
    unlock();
}
void function1(){
    lock();
	    //printf("Action function 1\n");
        //time_now("At:");
        // change reconfig 
        if(switch_for_hook==0){
            if(enable_ecoder_local_costmap!=0)
            enable_ecoder_local_costmap=stof_f(set_get_param("/"+mvibot_seri+"/move_base_flex/local_costmap/obstacles3/set_parameters","enabled","bool","0"));
            //
            if(enable_ecoder_global_costmap!=0)
            enable_ecoder_global_costmap=stof_f(set_get_param("/"+mvibot_seri+"/move_base_flex/global_costmap/obstacles3/set_parameters","enabled","bool","0"));
        }else{
            if(enable_ecoder_local_costmap!=1)
            enable_ecoder_local_costmap=stof_f(set_get_param("/"+mvibot_seri+"/move_base_flex/local_costmap/obstacles3/set_parameters","enabled","bool","1"));
            //
            if(enable_ecoder_global_costmap!=1)
            enable_ecoder_global_costmap=stof_f(set_get_param("/"+mvibot_seri+"/move_base_flex/global_costmap/obstacles3/set_parameters","enabled","bool","1"));
        }
        // add negative speed ....
        check_pose_costmap();
        pub_footprint_infor();
        pub_path_infor();
        // save the mission
    unlock();
}
void function2(){
    lock();
     position_robot=get_position("/map","/"+mvibot_seri+"/base_footprint");
        if(position_robot[0]==-1 & position_robot[1]==-1 & position_robot[2]==-1 & position_robot[3]==-1)
        tranfom_map_robot=0;
        else tranfom_map_robot=1;
        //
        if(tranfom_map_robot==1){
            collect_user_pathf();
            // save postion
            static string data_position;
            static string cmd;
            static float x,y,z,w;
            static tf::Quaternion q;
            //
            x=position_robot[0]; y=position_robot[1]; z=position_robot[2]; w=position_robot[3];
            q.setZ(z);
            q.setW(w);
            //
            data_position="";
            data_position=data_position+"initial_pose_x: "+to_string(x)+"\n";
            data_position=data_position+"initial_pose_y: "+to_string(y)+"\n";
            data_position=data_position+"initial_pose_a: "+to_string(tf::getYaw(q))+"\n";	
            cmd="";
            cmd=cmd+"echo "+"'"+data_position+"' > "+define_path+"param/position.yaml";
            system(cmd.c_str());
        }
    unlock();
}
void function3(){
    lock();
        if(motor_left_ready == 0 | motor_right_ready==0){
            if(action_mission==Active_){
                action_mission=Cancel_;
                send_history("error","EMG 2 motor so stop action mission");
            }
        }
        // update gpio
        input_user_status_2=input_user_status_1;
        input_user_status_1=input_user_status;
        for(int i=0;i<my_module.size();i++){
            my_module[i].input_user2=my_module[i].input_user1;
            my_module[i].input_user1=my_module[i].input_user;
        }
        // update battery soc
        battery_soc2=battery_soc1;
        battery_soc1=battery_soc;
        static variable_local battery_varialbe;
        battery_varialbe.name="battery";
        battery_varialbe.data=battery_soc;
        if(my_vars_local.var.size()==0){
            my_vars_local.add_var(battery_varialbe);
        }else my_vars_local.update_var(battery_varialbe);
        // check battery soc 
        if(battery_soc1!=-1 & battery_soc2 != -1 & battery_soc_set_mission!=-1){
            if(battery_soc1<=battery_soc_set_mission & battery_soc2>battery_soc_set_mission){
                want_to_charge=1;
                send_history("warning","Baterry lower than target charge, wait change to mode mission battery charge");
            }
        }
        //
        static int res;
        //
        my_vars_local.print();
        //
        static int action_mode_mission_f;
        if(action_mode_mission==Mission_normal_){
            //
            if(action_mode_mission_f==Mission_charge_battery_) my_mission_charge_battery.reset();
            //
            if(action_mission!=Active_){
                res=my_multiple_mission.action(Cancel_);
                //
                if(motor_left_ready != 0 & motor_right_ready!=0){
                    if(res==Wake_up_){
                        if(action_mission!=Error_){
                            action_mission=Active_;
                            send_history("normal","Start mission action normal by condition wake up. Name mission: "+my_multiple_mission.get_name_mission_active());
                        }
                        else{
                            action_mission=Cancel_;
                            send_history("normal","Reset error mission action normal by condition wake up. Name mission: "+my_multiple_mission.get_name_mission_active());
                        }
                    }
                    if(res==Continue_){
                        if(my_multiple_mission.num_mission_action!=-1){
                            if(action_mission!=Error_){
                                action_mission=Active_;
                                send_history("normal","Start mission action normal by condition continue. Name misison: "+my_multiple_mission.get_name_mission_active());
                            }
                            else{
                                action_mission=Cancel_;
                                send_history("normal","Reset error mission action normal by condition continue. Name mission: "+my_multiple_mission.get_name_mission_active());
                            }
                        }
                    }
                    //
                }
            }else{
                res=my_multiple_mission.action(Active_);
                action_mission=res;
                if(res==Finish_) send_history("normal","Finish mission action normal: "+my_multiple_mission.get_name_mission_active());   
                if(res==Error_)  send_history("error","Error mission normal, more detail: "+my_multiple_mission.get_infor(0));
            }
            // change to battery mission
            if(action_mission!=Active_ & action_mission!=Error_ & motor_left_ready == 1 & motor_right_ready== 1  & my_multiple_mission.num_mission_action==-1){
                // scan start mission battery
                res=my_mission_charge_battery.action(Cancel_);
                if(res==Wake_up_) want_to_charge=1;
                //
                if(want_to_charge==1){
                    if(res!=Wake_up_) send_history("normal","Start mission action charge battery by target charge. Name mission:"+my_mission_charge_battery.name_mission);         
                    else send_history("normal","Start mission action charge battery by condition wake up. Name mission: "+my_mission_charge_battery.name_mission);
                    action_mode_mission=Mission_charge_battery_;
                    action_mission=Active_;
                    my_mission_charge_battery.reset();
                }
            }
        }
        else if(action_mode_mission==Mission_charge_battery_){
            if(action_mission==Active_){
                res=my_mission_charge_battery.action(Active_);
                if(res==Finish_) {
                    action_mission=Finish_;
                    action_mode_mission=Mission_normal_;
                    want_to_charge=0;
                    my_mission_charge_battery.reset();
                    send_history("normal","Finish  action mission charge battery. Name mission: "+my_mission_charge_battery.name_mission);
                    send_history("normal","Change to mode action mission normal");      
                }else if(res==Cancel_){
                    send_history("normal","Pause mission action charge battery by condition stop. More detail: "+my_mission_charge_battery.get_infor(1));
                }else if(res==Error_){
                    send_history("error","Error mission action charge battery. More detail: "+my_mission_charge_battery.get_infor(1));

                }
                action_mission=res;
            }else{
                res=my_mission_charge_battery.action(Cancel_);
                if(res==Wake_up_ | res==Continue_){
                    if(action_mission!=Error_){
                        action_mission=Active_;
                        if(res==Wake_up_)  send_history("normal","Start mission action charge battery by condition wake up. Name mission: "+my_mission_charge_battery.name_mission);
                        if(res==Continue_) send_history("normal","Start mission action charge battery by condition continue. Name mission: "+my_mission_charge_battery.name_mission);
                    }
                    else{
                        action_mission=Cancel_;
                        if(res==Wake_up_)  send_history("normal","Reset error  mission action charge battery by condition wake up. Name mission: "+my_mission_charge_battery.name_mission);
                        if(res==Continue_) send_history("normal","Reset error  mission action charge battery by condition continue. Name mission: "+my_mission_charge_battery.name_mission);
                    }
                }
            }
        }
        action_mode_mission_f=action_mode_mission;
        //
        static int action_mission_f;
        if(action_mission==Error_){
            my_mission_error.action(Active_);
        }else{
            if(action_mission_f==Error_){
             my_mission_error.reset();
            }
        }
        action_mission_f=action_mission;
        // control led
        set_led();
        // control sound
        set_sound();
        //
        pub_mission_action_infor();
        pub_local_variable();
        pub_mission_memory();
    unlock();
}