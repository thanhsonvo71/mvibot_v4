#include "../libary/libary_basic.h"
using namespace std;
#if !defined(read_file_define)
 	string read_file(string path){
		ifstream file;
	    string str,data;
	    file.open(path);
		data="";
		while (std::getline(file, str))
		{
		    	data=data+str;
		}
	    file.close();
		return data;
	}
    #define read_file_define 1
#endif
