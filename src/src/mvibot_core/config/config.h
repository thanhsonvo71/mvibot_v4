#include"../../common/libary/libary_basic.h"
#include"../../common/libary/libary_ros.h"
#include"../../common/hardware/hardware.h"
#include"../../common/stof/stof.h"
#include"../../common/exec/exec.h"
#include"../mvibot_core_init.h"
using namespace std;
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
void pub_robot_status(){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub_robot_status = n.advertise<std_msgs::String>("/"+mvibot_seri+"/robot_status",1);
    static ros::Publisher pub_robot_status_string = n1.advertise<std_msgs::String>("/robot_status_string",1);
    static float creat_fun=0;
    static vector<string> list={"mode","ip_node","ip_master"};
    static vector<string> param;
    static std_msgs::String msg;
        if(creat_fun==1){
                pub_robot_status.publish(msg);
                pub_robot_status_string.publish(msg);
        }else{
            creat_fun=1;
            param.resize(list.size());
            for(int i=0;i<list.size();i++){
                try{
                        std::ifstream file(define_path+"config/"+list[i]);
                        static std::string str; 
                        static std::string data;
                        data="";
                        while (std::getline(file, str))
                        {
                            // Process str
                            data=str;
                        }
                        file.close();
                        param[i]=data;
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
            }
            msg.data=mvibot_seri;
            for(int i=0;i<list.size();i++){
                msg.data=msg.data+"|";
                msg.data=msg.data+list[i]+":"+param[i];
            }
        }
}
void pub_robot_config(){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub = n.advertise<std_msgs::String>("/robot_config_status_string",1);
    static float creat_fun=0;
        if(creat_fun==1){
                static std_msgs::String msg;
                msg.data=robot_config_string;
                pub.publish(msg);
        }else creat_fun=1;
}
void robot_load_config(){
        static string config;
        static string my_port;
        //
        robot_config_string=mvibot_seri+"|";
        // robot config 
        config=load_file("robot_R");
        if(config!="-1") R=stof_f(config);
        if(config!="-1") robot_config_string+="robot_R:"+config+"|";
        //
        config=load_file("robot_L");
        if(config!="-1") L=stof_f(config);
        if(config!="-1") robot_config_string+="robot_L:"+config+"|";
        //
        config=load_file("robot_gear");
        if(config!="-1") gear=stof_f(config);
        if(config!="-1") robot_config_string+="robot_gear:"+config+"|";
        //
        config=load_file("robot_ax");
        if(config!="-1") ax=stof_f(config);
        if(config!="-1") robot_config_string+="robot_ax:"+config+"|";
        //
        config=load_file("robot_aw");
        if(config!="-1") aw=stof_f(config);
        if(config!="-1") robot_config_string+="robot_aw:"+config+"|";
        //
        config=load_file("robot_vmax");
        if(config!="-1") v_max=stof_f(config);
        if(config!="-1") robot_config_string+="robot_vmax:"+config+"|";
        //
        config=load_file("robot_wmax");
        if(config!="-1") w_max=stof_f(config);
        if(config!="-1") robot_config_string+="robot_wmax:"+config+"|";
        //
        config=load_file("robot_volume");
        if(config!="-1") volume=stof_f(config);
        if(config!="-1") robot_config_string+="robot_volume:"+config+"|";
        //
        config=load_file("robot_low_battery");
        if(config!="-1") robot_config_string+="robot_low_battery:"+config+"|";
        //
        config=load_file("robot_type_connect");
        if(config!="-1") {
            robot_config_string+="robot_type_connect:"+config+"|";
            if(config=="lan") my_port=load_file("lan_port");
            if(config=="wifi") my_port=load_file("wifi_port");
        }
        // camera seri config
        config=load_file("serial_camera1");
        if(config!="-1") robot_config_string+="serial_camera1:"+config+"|";
        //
        config=load_file("serial_camera2");
        if(config!="-1") robot_config_string+="serial_camera2:"+config+"|";  
        // lan cònfig
        config=load_file("lan_type");
        if(config!="-1") robot_config_string+="lan_type:"+config+"|";  
        config=load_file("lan_ipv4");
        if(config!="-1") robot_config_string+="lan_ipv4:"+config+"|";  
        config=load_file("lan_ipv4_gateway");
        if(config!="-1") robot_config_string+="lan_ipv4_gateway:"+config+"|";  
        config=load_file("lan_ipv4_dns");
        if(config!="-1") robot_config_string+="lan_ipv4_dns:"+config+"|";  
        // wifi cònfig
        config=load_file("wifi_type");
        if(config!="-1") robot_config_string+="wifi_type:"+config+"|";  
        config=load_file("wifi_ssid");
        if(config!="-1") robot_config_string+="wifi_ssid:"+config+"|";
        config=load_file("wifi_password");
        if(config!="-1") robot_config_string+="wifi_password:"+config+"|";
        config=load_file("wifi_ipv4");
        if(config!="-1") robot_config_string+="wifi_ipv4:"+config+"|";  
        config=load_file("wifi_ipv4_gateway");
        if(config!="-1") robot_config_string+="wifi_ipv4_gateway:"+config+"|";  
        config=load_file("wifi_ipv4_dns");
        if(config!="-1") robot_config_string+="wifi_ipv4_dns:"+config+"|";  
        //  operating config
        config=load_file("mode");
        if(config!="-1") robot_config_string+="mode:"+config+"|";  
        //
        config=load_file("ip_master");
        if(config!="-1") robot_config_string+="ip_master:"+config+"|";  
        //
        config=load_file("is_master");
        if(config!="-1") robot_config_string+="is_master:"+config+"|";  
        //
        config="ip -4 addr show "+my_port+" | grep -oP '(?<=inet\\s)\\d+(\\.\\d+){3}')";
        config=exec(config.c_str());
        config.erase(std::remove(config.begin(), config.end(), '\n'), config.cend());
        robot_config_string+="ip_node:"+config;//+"|"; 
        /*
        config=load_file("ip_node");
        if(config!="-1") robot_config_string+="ip_node:"+config;//+"|";  
        //*/
        
}