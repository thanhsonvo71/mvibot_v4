#include "../libary/libary_basic.h"
using namespace std;
#if !defined(string_Iv2_define)
    class string_Iv2;
    class string_Iv2{
        public:
            vector<string> data1;
            vector<string> data2; 
            void detect(string data,string data_start, string data_complies, string data_end);
            void print();
    };
    void string_Iv2::detect(string data,string data_start, string data_complies, string data_end){
        data1.resize(0);
        data2.resize(0);
        //
        if(data_start !="" && data_end !=""){
            for(int i=0;i<data.length();i++){
                static string data_;
                data_=data.substr(i,data_start.length());
                if(data_==data_start){
                    for(int j=i+1;j<data.length();j++){
                        data_=data.substr(j,data_end.length());
                        if(data_==data_end){
                            data_=data.substr(i+data_start.length(),j-(i+data_start.length()));
                            if(data_complies==""){
                                int size;
                                size=data1.size();
                                //
                                data1.resize(size+1);
                                data2.resize(size+1);
                                data1[size]=data_;
                                data2[size]="";
                                //
                            }else{
                                int size;
                                size=data1.size();
                                //
                                data1.resize(size+1);
                                data2.resize(size+1);
                                //
                                static int check;
                                check=0;
                                for(int k=0;k<data_.length();k++){
                                    static string data2_;
                                    data2_=data_.substr(k,data_complies.length());
                                    if(data2_== data_complies){
                                        check=1;
                                        k=k+data_complies.length()-1;
                                    }else{
                                        if(check==0) data1[size]=data1[size]+data_[k];
                                        else if(check==1) data2[size]=data2[size]+data_[k];
                                    }
                                }                       
                            }
                            i=j+data_end.length()-1;
                            break;
                        }
                    }
                }
            }
        }else{
            data1.resize(1);
            data2.resize(1);
            for(int i=0;i<data.length();i++){
                static string data_;
                data_=data.substr(i,data_complies.length());
                //
                static int size;
                size=data1.size();
                if(data_==data_complies){
                    data1.resize(size+1);
                    data2.resize(size+1);
                    i=i+data_complies.length()-1;
                }else{
                    data1[size-1]=data1[size-1]+data[i];
                    data2[size-1]="";
                }
            }
        }
        //
    }
    void string_Iv2::print(){
        for(int i=0;i<data1.size();i++){
            cout<<"Data1: "<<data1[i]<<endl;
            cout<<"Data2: "<<data2[i]<<endl;
        }
    }
    #define string_Iv2_define 1
#endif