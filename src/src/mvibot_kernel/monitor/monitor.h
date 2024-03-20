#include "../../common/exec/exec.h"
#include "../../common/string_Iv2/string_Iv2.h"
using namespace std;
//
extern vector<string> list_monitor;
//
void get_list_monitor(){
    // check list output display
    static string_Iv2 string_suport;
    static string data;
    list_monitor.resize(0);
    data=exec("ls /sys/class/drm/");
    string_suport.detect(data,"","\n","");
    for(int i=0;i<string_suport.data1.size();i++){
        if(string_suport.data1[i]!=""){
            list_monitor.resize(list_monitor.size()+1);
            list_monitor[list_monitor.size()-1]=string_suport.data1[i];
            cout<<list_monitor[list_monitor.size()-1]<<endl;
        }
    }
}
int check_monitor_connect(){
    int check_return;
    check_return=0;
    try{
        static string cmd;
        static string out_put;
        cmd="cat /sys/class/drm";
        for(int i=0;i<list_monitor.size();i++){    
            out_put=exec(cmd+"/"+list_monitor[i]+"/status");
            cout<<out_put<<endl;
            if(out_put=="connected\n"){
                cout<<"Some one is connect to output display..."<<endl;
                check_return=1;
            }
        }
    }catch(const std::exception& e){
        cout<<"Can't check monitor connect"<<endl;
    }
    return check_return;
}