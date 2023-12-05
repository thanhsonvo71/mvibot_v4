#include "../libary/libary_basic.h"
using namespace std;
#if !defined(stof_define)
   float stof_f(string data){
        static float value_return;
        try
        {
            value_return=stof(data);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            value_return=-1;
        }
        return value_return;  
    }
    double stod_f(string data){
        static double value_return;
        try
        {
            value_return=stod(data);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            value_return=-1;
        }
        return value_return;  
    }
    #define stof_define 1
#endif

