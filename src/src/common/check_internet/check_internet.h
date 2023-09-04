#include "../libary/libary_basic.h"
#include "../exec/exec.h"

using namespace std;
//
static string string_not_connect="curl: (6) Could not resolve host: www.goole.com\n";
int check_internet(){
    static string data;
    data=exec("curl www.goole.com");
    if(data==string_not_connect) return 0;
    else return 1;
}