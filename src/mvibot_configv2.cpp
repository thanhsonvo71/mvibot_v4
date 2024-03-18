#include "src/common/libary/libary_basic.h"
#include "src/common/libary/libary_ros.h"
#include "src/common/thread/thread.h"
using namespace  std;
std::string mvibot_seri;
// settime process
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
void set_config(const std_msgs::String& msg){
	static int k=0;
	printf("%s\n",msg.data.c_str());
	static ofstream myfile;
	for(int i=0;i<msg.data.length();i++){
		if(msg.data[i]=='('){
			name="";
			data="";
			k=0;
			for(int j=i+1;j<msg.data.length();j++){
				if(msg.data[j]==')') {
					i=j;
					break;
				}
				else {
					if(msg.data[j]!='|'){
						if(k==0) name=name+msg.data[j];
						else data=data+msg.data[j];
					}else k++;
				}
				i=j;
			}
		}
		printf("%s <--- %s\n",name.c_str(),data.c_str());
		myfile.open("/home/mvibot/catkin_ws/src/mvibot_v4/config/"+name);
		myfile<<data;
		myfile.close();
		if(name=="ip_master"){
			static string cmd;
			cmd="";
			cmd=cmd+"echo export const ip = '"+data+"' > "+"/home/mvibot/catkin_ws/src/mvibot_v4/interface_mvibot_v2/public/ip.js"; 
			//system(cmd.c_str());
			myfile.open("/home/mvibot/catkin_ws/src/mvibot_v4/interface_mvibot_v2/public/ip.js");
			myfile<<"export const ip='"+data+"'";
            myfile.close();
		}
	}
}
string read_config(string name){
	static string mydata="";
	static ifstream myfile;
	myfile.open("/home/mvibot/catkin_ws/src/mvibot_v4/config/"+name);
    std::getline(myfile,mydata);
	myfile.close();
	mydata='('+name+'|'+mydata+')';
	return mydata;	
}
// void function1(){
// }
// void function2(){
// }
// void * process1(void * nothing){
// 	process("Process A",ts_process1,0,function1);
// }
// void * process2(void * nothing){
// 	process("Process B",ts_process2,0,function2);
// }
int main(int argc, char** argv) {
    int res;
    //
    ros::init(argc, argv, "mvibot_config");
    ros::NodeHandle nh("~");
    nh.getParam("mvibot_seri", mvibot_seri);
    //res=pthread_create(&p_process1,NULL,process1,NULL);
    //res=pthread_create(&p_process2,NULL,process2,NULL);
    printf("Done\n");
    ros::NodeHandle n1;
    ros::Subscriber sub1 = n1.subscribe("/"+mvibot_seri+"/set_config", 1, set_config);    
    ros::spin(); 
    return 0;
}



