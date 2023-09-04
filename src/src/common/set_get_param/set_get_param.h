#include"../libary/libary_ros.h"
#include"../ros_timenow/ros_timenow.h"
#include"../stof/stof.h"
#include"../stoi/stoi.h"
using namespace std;
#if !defined(set_get_param_define)
    string set_get_param(string command,string name, string type, string value){
        //set
        static dynamic_reconfigure::ReconfigureRequest srv_req;
        static dynamic_reconfigure::ReconfigureResponse srv_resp;
        static dynamic_reconfigure::Config conf;
        //
        static dynamic_reconfigure::DoubleParameter double_param;
        static dynamic_reconfigure::IntParameter int_param;
        static dynamic_reconfigure::BoolParameter bool_param;
        static dynamic_reconfigure::StrParameter string_param;
        static dynamic_reconfigure::Group param;
        //
        conf.bools.resize(0);
        conf.ints.resize(0);
        conf.strs.resize(0);
        conf.doubles.resize(0);
        if(value!=""){
            if(type=="string"){
                string_param.name=name;
                string_param.value=value;
                conf.strs.push_back(string_param);
            }else if(type=="int"){
                int_param.name=name;
                int_param.value=stoi_f(value);
                conf.ints.push_back(int_param);
            }else if(type=="double"){
                double_param.name=name;
                double_param.value=(double)((long long)(stof_f(value)*1e5))/1e5;//(double)(stof_f(value));
                conf.doubles.push_back(double_param);
            }else if(type=="bool"){
                bool_param.name=name;
                if(value=="1") bool_param.value=1;
                else bool_param.value=0;
                conf.bools.push_back(bool_param);
            }
        }
        srv_req.config = conf;
        ros::service::call(command, srv_req, srv_resp);
        // get
        static string value_return;
        value_return="-1";
        if(type=="string"){
            for(int i=0;i<srv_resp.config.strs.size();i++){
                if(srv_resp.config.strs[i].name==name){
                    value_return=srv_resp.config.strs[i].value;
                    break;
                }
            }
        }else if(type=="int"){
            for(int i=0;i<srv_resp.config.ints.size();i++){
                if(srv_resp.config.ints[i].name==name){
                    value_return=to_string(srv_resp.config.ints[i].value);
                    break;
                }
            }
        }else if(type=="double"){
            for(int i=0;i<srv_resp.config.doubles.size();i++){
                if(srv_resp.config.doubles[i].name==name){
                    value_return=to_string(srv_resp.config.doubles[i].value);
                    break;
                }
            }

        }else if(type=="bool"){
            for(int i=0;i<srv_resp.config.bools.size();i++){
                if(srv_resp.config.bools[i].name==name){
                    if(srv_resp.config.bools[i].value==1) value_return="1";
                    else value_return="0";
                    break;
                }
            }
        }
        return value_return;
    }
    #define set_get_param_define 1
#endif
