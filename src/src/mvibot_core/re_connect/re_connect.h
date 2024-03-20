#include"../../common/libary/libary_ros.h"
#include"../../common/libary/libary_basic.h"
#include"../../common/exec/exec.h"
//
using namespace std;
extern int master_check_status;
extern int n_re_connect;
//
void re_connect(){
    if(master_check_status==1){
        n_re_connect=0;
        master_check_status=0;
    }else{
        n_re_connect++;
        if(n_re_connect>=70){
            n_re_connect=10;
            //exec("nmcli connection up mvibot_cardwifi &");
            system("nmcli connection up mvibot_cardwifi &");
        }
    }
}