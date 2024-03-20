#include "../libary/libary_basic.h"
using namespace std;
#if !defined(send_mail_define)
    void send_mail(string mail_addrees_to,string title, string msg, string path_to_file){
        static string cmd;
        cmd="";
        if(path_to_file!=""){
            // cmd=cmd+"echo "+msg+" | mail -s '"+title+"' "+mail_addrees_to;

        }else{
            cmd=cmd+"echo "+msg+" | mail -s '"+title+"' "+mail_addrees_to;
            system(cmd.c_str());
            //cout<<cmd<<endl;
        }
    }
    #define send_mail_define 1
#endif

