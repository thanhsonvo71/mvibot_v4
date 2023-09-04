#include "../mvibot_kernel_init.h"
#include "../../common/exec/exec.h"
using namespace std;
//
int check_security(){
    static string string_return;
    static int match_security=1;
    match_security=1;
    //
    string_return=exec("sudo dmidecode -s baseboard-manufacturer");
    if(string_return!=board_manufacturer+"\n") match_security=0;
    string_return=exec("sudo dmidecode -s baseboard-product-name");
    if(string_return!=board_product_name+"\n") match_security=0;
    string_return=exec("sudo dmidecode -s baseboard-version");
    if(string_return!=board_version+"\n") match_security=0;
    string_return=exec("sudo dmidecode -s baseboard-serial-number");
    if(string_return!=board_serial+"\n") match_security=0;
    //
    return match_security;
}
void security_code(){
    // protect code here
    exit(0);
}