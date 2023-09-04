#include "../libary/libary_basic.h"
using namespace std;
#if !defined(stoi_define)
   int stoi_f(string data){
        static int value_return;
        try
        {
            value_return=stoi(data);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            value_return=-1;
        }
        return value_return;
    }
    #define stoi_define 1
#endif

