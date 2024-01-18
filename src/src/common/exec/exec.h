#include"../libary/libary_basic.h"
using namespace std;
#if !defined(exec_define)
    std::string exec(string cmd) {
        static FILE *fp;
        static int status;
        static char path[999];
        static string string_return;
        string_return="";
        fp = popen(cmd.c_str(), "r");
        if (!fp){    
            pclose(fp);
            return "popen failed!";
        }else{
            while (fgets(path, 999, fp) != NULL)
            {
                string_return+=string(path);
            }
        }
        pclose(fp);
        return string_return;
    }
    #define exec_define 1
#endif