#include "src/common/libary/libary_basic.h"
#include "src/common/libary/libary_ros.h"
#include "src/common/thread/thread.h"
#include "src/common/string_Iv2/string_Iv2.h"
#include "src/common/exec/exec.h"
#include "src/common/read_file/read_file.h"
#include "src/common/hardware/hardware.h"
#include "src/common/stof/stof.h"
using namespace  std;
std::string mvibot_seri;
int scan_wifi_request=1;
int connect_wifi_request=0;
class wifi{
	public:
		string ssid;
		string signal;
		string active;
		string security;
};
class n_wifi{
	public:
		vector<wifi> n_wifi_;
		void add_wifi(string ssid, string signal, string active, string security);
		void print();
		string string_msg();
};
n_wifi my_wifi;
string wifi_ssid_to_connect;
string wifi_pw_to_connect;
// settime process
static pthread_mutex_t process_mutex=PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;	
long double ts_process1=1.0; //time set for process1
long double ts_process2=5.0; //time set for process2
long double ts_process3=2.0;
long double n2,over;
// var for process
static pthread_t p_process1;
static pthread_mutex_t process1_mutex;
static pthread_t p_process2;
static pthread_t p_process3;
static string name="";
static string data="";
//
void pub_robot_list_wifi(string data){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/robot_list_wifi",1);
    static float creat_fun=0;
	if(creat_fun==1){
			static std_msgs::String msg;
			msg.data=data;
			pub.publish(msg);
	}else creat_fun=1;
}
//
void lock();
void unlock();
//
void set_configf(const std_msgs::String& msg){
	lock();
		static ofstream myfile;
		static string_Iv2 data;
		data.detect(msg.data,"(","|",")");
		for(int i=0;i<data.data1.size();i++){
			printf("%s <--- %s\n",data.data1[i].c_str(),data.data2[i].c_str());
			myfile.open(define_path+"config/"+data.data1[i]);
			myfile<<data.data2[i];
			myfile.close();
			//
			// if(data.data1[i]=="ip_master"){
			// 	myfile.open("/home/mvibot/interface_mvibot_v2/public/ip.js");
			// 	myfile<<"export const ip='"+data.data1[2]+"'";
			// 	myfile.close();
			// }
		}
	unlock();
}
void scan_wifif(const std_msgs::String& msg){
	lock();
		if(scan_wifi_request==0) scan_wifi_request=1;
	unlock();
}
void new_connect_wifif(const std_msgs::String& msg){
	lock();
		if(connect_wifi_request==0) connect_wifi_request=1;
		static string_Iv2 data;
		data.detect(msg.data,"","|","");
		//
		wifi_ssid_to_connect="";
		wifi_pw_to_connect="";
		if(data.data1.size()==2){
			wifi_ssid_to_connect=data.data1[0];
			wifi_pw_to_connect=data.data1[1];
		}
		//
	unlock();
}
void new_connect_wifi_manualf(const std_msgs::String& msg){
	lock();
	//
		static string_Iv2 data;
		data.detect(msg.data,"","|","");
		//
		wifi_ssid_to_connect="";
		wifi_pw_to_connect="";
		if(data.data1.size()==2){
			wifi_ssid_to_connect=data.data1[0];
			wifi_pw_to_connect=data.data1[1];
		}
		// save data here
	//
	unlock();
}
//
void function1();
void function2();
void * process1(void * nothing){
	char name[]="Process A";
	process(name,ts_process1,0,function1);
	void *value_return;
    return value_return;
}
void * process2(void * nothing){
	char name[]="Process B";
	process(name,ts_process2,0,function2);
	void *value_return;
    return value_return;
}
int main(int argc, char** argv) {
    int res;
    //
    ros::init(argc, argv, "mvibot_config");
    ros::NodeHandle nh("~");
    string test;
	//
	nh.getParam("mvibot_seri", mvibot_seri);
    res=pthread_create(&p_process1,NULL,process1,NULL);
    //res=pthread_create(&p_process2,NULL,process2,NULL);
    printf("Done\n");
    ros::NodeHandle n1,n2,n3,n4;
    ros::Subscriber sub1 = n1.subscribe("/"+mvibot_seri+"/set_config", 1, set_configf);    
	ros::Subscriber sub2 = n2.subscribe("/"+mvibot_seri+"/scan_wifi", 1, scan_wifif); 
	ros::Subscriber sub3 = n3.subscribe("/"+mvibot_seri+"/new_connect_wifi", 1, new_connect_wifif); 
	ros::Subscriber sub4 = n4.subscribe("/"+mvibot_seri+"/new_connect_wifi_manual", 1, new_connect_wifi_manualf); 
    ros::spin(); 
    return 0;
}
//
void n_wifi::add_wifi(string ssid, string signal, string active, string security){
	static int is_have;
	is_have=0;
	for(int i=0;i<n_wifi_.size();i++){
		if(n_wifi_[i].ssid==ssid){
			is_have=1;
			if(stof_f(signal)>stof_f(n_wifi_[i].signal)){
				n_wifi_[i].signal=signal;
			}
			if(active=="yes"){
				n_wifi_[i].active=active;
			}
			break;
		}
	}
	//
	if(is_have==0){
		n_wifi_.resize(n_wifi_.size()+1);
		n_wifi_[n_wifi_.size()-1].ssid=ssid;
		n_wifi_[n_wifi_.size()-1].signal=signal;
		n_wifi_[n_wifi_.size()-1].active=active;
		n_wifi_[n_wifi_.size()-1].security=security;
	}
}
void n_wifi::print(){
	for(int i=0;i<n_wifi_.size();i++){
		cout<<"SSID:";
		cout<<n_wifi_[i].ssid;
		cout<<"\tsignal:"<<n_wifi_[i].signal;
		cout<<"\tactive:"<<n_wifi_[i].active;
		cout<<"\tsecurity:"<<n_wifi_[i].security<<endl;
	}
}
string n_wifi::string_msg(){
	static string string_return;
	string_return="";
	for(int i=0;i<n_wifi_.size();i++){
		string_return=string_return+"(";
		string_return=string_return+"~ssid="+n_wifi_[i].ssid+"~";
		string_return=string_return+"~signal="+n_wifi_[i].signal+"~";
		string_return=string_return+"~active="+n_wifi_[i].active+"~";
		string_return=string_return+"~security="+n_wifi_[i].security+"~";
		string_return=string_return+")";
	}
return string_return;
}
//
void lock(){
    pthread_mutex_lock(&process_mutex);
}
void unlock(){
    pthread_mutex_unlock(&process_mutex);
}
void function1(){
	static string out_put;
	static string_Iv2 data;
	if(connect_wifi_request==0){
		if(scan_wifi_request==1){
			out_put="";
			scan_wifi_request=0;
			out_put=exec("sudo nmcli -t -f SSID,signal,active,security device wifi list --rescan yes");
		}else{
			//out_put=exec("sudo nmcli -t -f SSID,signal,active device wifi list	");
		}
		// process data
		data.detect(out_put,"","\n","");
		my_wifi.n_wifi_.resize(0);
		for(int i=0;i<data.data1.size();i++){
			if(data.data1[i]!=""){
				static string_Iv2 data2;
				data2.detect(data.data1[i],"",":","");
				if(data2.data1.size()==4){
					if(data2.data1[0]!=""){
						my_wifi.add_wifi(data2.data1[0],data2.data1[1],data2.data1[2],data2.data1[3]);
					}
				}
			}
		}
		// 
		cout<<"my list wifi"<<endl;
		my_wifi.print();
		pub_robot_list_wifi(my_wifi.string_msg());
	}else{
		cout<<"Connect new wifi"<<endl;
		if(wifi_ssid_to_connect!=""){
			static string wifi_port;
			wifi_port=read_file(define_path+"config/"+"wifi_port");
			//
			static string cmd;
			cmd="";
			cmd=cmd+"sudo nmcli connection delete mvibot_cardwifi";
			system(cmd.c_str());
			//
			cmd="";
			cmd=cmd+"sudo nmcli connection add ifname "+wifi_port+" con-name mvibot_cardwifi type wifi ssid "+wifi_ssid_to_connect;
			system(cmd.c_str());
			//
			if(wifi_pw_to_connect!=""){
				cmd="";
				cmd=cmd+"sudo nmcli connection modify mvibot_cardwifi 802-11-wireless-security.key-mgmt WPA-PSK 802-11-wireless-security.psk "+wifi_pw_to_connect;
				system(cmd.c_str());
			}
			//
			cmd="sudo nmcli connection up mvibot_cardwifi";
			system(cmd.c_str());
			//
		}
		connect_wifi_request=0;
		scan_wifi_request=1;
		sleep(1);
	}
	
}
void function2(){
}