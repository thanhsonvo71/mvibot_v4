#include "../mvibot_kernel_init.h"
#include "../../common/exec/exec.h"
#include "../../common/string_Iv2/string_Iv2.h"
#include "../../common/send_mail/send_mail.h"

using namespace std;
void backup_file(string name_file,string value){
    static string cmd;
    cmd="";
    cmd=cmd+"echo "+value+" > "+define_path+name_file;
    system(cmd.c_str());
}
int backup(){
    //
    cout<<"Backup program robot"<<endl;
    exec("cd /home/mvibot/catkin_ws/devel/lib/mvibot/ && sudo unzip -o -P mvibot_"+board_serial+name_seri_fix+" mvibot_backup.zip");
    cout<<"cd /home/mvibot/catkin_ws/devel/lib/mvibot/ && unzip -o -P mvibot_"+board_serial+name_seri_fix+" mvibot_backup.zip"<<endl;
    //
    cout<<"Backup web interface"<<endl;
    //
    cout<<"reconfig robot......"<<endl;
    backup_file("config/name_seri",name_seri_fix);
    // robot config
    backup_file("config/robot_R","0.0805");
    backup_file("config/robot_L","0.594");
    backup_file("config/robot_gear","25");
    backup_file("config/robot_vmax","0.5");
    backup_file("config/robot_wmax","0.314");
    backup_file("config/robot_ax","1.0");
    backup_file("config/robot_aw","3.0");
    backup_file("config/robot_volume","100");
    backup_file("config/robot_low_battery","-1");
    backup_file("config/robot_type_connect","lan");
    // lan config
    backup_file("config/lan_port","enp0s31f6");
    backup_file("config/lan_type","auto");
    backup_file("config/lan_ipv4","");
    backup_file("config/lan_ipv4_gateway","");
    backup_file("config/lan_ipv4_dns","");
    // wifi config
    backup_file("config/wifi_port","wlp0s20f3");
    backup_file("config/wifi_type","hots_pot");
    backup_file("config/wifi_ipv4","");
    backup_file("config/wifi_ipv4_gateway","");
    backup_file("config/wifi_ipv4_dns","");
    backup_file("config/wifi_ssid","");
    backup_file("config/wifi_password","");
    // operating config
    backup_file("config/is_master","yes");
    backup_file("config/ip_master","192.168.0.200");
    backup_file("config/ip_node","192.168.0.200");
    backup_file("config/mode","slam");
    // reset mission
    backup_file("param/mission.yaml","");
    backup_file("param/mission_error.yaml","");
    backup_file("param/mission_charge_battery.yaml","");
    //
    send_mail(mail_master,name_seri_fix+" Backup Software",name_seri_fix+" is backup at `date`","");
    //
    return 1;
}