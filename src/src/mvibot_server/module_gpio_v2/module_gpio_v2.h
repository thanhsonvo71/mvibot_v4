#include "../mvibot_server_init.h"
using namespace std;
void get_module_gpio_v2_list(){
    try{
        free_res();
        res=stmt->executeQuery("SELECT * from `my_module_gpio_v2`");
        stmt->execute("UPDATE my_module_gpio_v2 set status=0");
        stmt->execute("UPDATE my_module_gpio_v2 set output_user_set_string_fesp=''");
        while (res->next()) {
            static int is_have=0,is_have2;
            is_have=0;
            is_have2=0;
            for(int i=0;i<my_module_gpio_v2s.size();i++){
                if(res->getString("name_seri")==my_module_gpio_v2s[i].name_seri){
                    //
                    my_module_gpio_v2s[i].my_node->input_user_status_string=res->getString("input_status_string");
                    my_module_gpio_v2s[i].my_node->output_user_status_string=res->getString("output_status_string");
                    if(res->getString("output_user_set_string_fesp") != ""){
                        pub_output_user_set_string(res->getString("output_user_set_string_fesp"));
                    }
                    //
                    if(stoi(res->getString("status"))==0) is_have=2;
                    else is_have=1;
                    break;
                }
            }
            for(int i=0;i<my_robots.size();i++){
                if(res->getString("name_seri")==my_robots[i].name_seri){
                    if(is_have==1) my_robots[i].time_out=0;
                    is_have2=1;
                    break;
                }
            }
            if(is_have==0){
                my_module_gpio_v2s.resize(my_module_gpio_v2s.size()+1);
                my_module_gpio_v2s[my_module_gpio_v2s.size()-1].name_seri=res->getString("name_seri");
                my_module_gpio_v2s[my_module_gpio_v2s.size()-1].my_node= new node;
                my_module_gpio_v2s[my_module_gpio_v2s.size()-1].my_node->name_node=res->getString("name_seri");
                my_module_gpio_v2s[my_module_gpio_v2s.size()-1].my_node->init();
            }
            if(is_have2==0){
                my_robots.resize(my_robots.size()+1);
                my_robots[my_robots.size()-1].name_seri=res->getString("name_seri");
                my_robots[my_robots.size()-1].type="module_gpio";
                my_robots[my_robots.size()-1].update_database=0;
                //
                my_robots[my_robots.size()-1].node=new node_v2_3;
                my_robots[my_robots.size()-1].node->name_seri=my_robots[my_robots.size()-1].name_seri;   
                my_robots[my_robots.size()-1].node->init();
                printf("Add new: %s \n" , my_robots[my_robots.size()-1].name_seri.c_str());
            }
            //

        }
    }catch (sql::SQLException &e) {
        cout << "# ERR: " << e.what();
        cout << " (MySQL error code: " << e.getErrorCode();
        cout << ", SQLState: " << e.getSQLState() << " )" << endl;
    }
}
void update_module_gpio_v2(){
    for(int i=0;i<my_module_gpio_v2s.size();i++){
        //
        try{
            pub_input_user_status_string(my_module_gpio_v2s[i].my_node->input_user_status_string);
            pub_output_user_status_string(my_module_gpio_v2s[i].my_node->output_user_status_string);
            // update msg set gpio
            if(my_module_gpio_v2s[i].my_node->output_user_set_string!=""){
                static string cmd;
                cmd="UPDATE my_module_gpio_v2 set output_user_set_string='"+my_module_gpio_v2s[i].my_node->output_user_set_string+"'";
                cmd=cmd+" where name_seri='"+my_module_gpio_v2s[i].my_node->name_node+"'";
                stmt->execute(cmd);
                my_module_gpio_v2s[i].my_node->output_user_set_string="";
            }
            if(my_module_gpio_v2s[i].my_node->output_user_set_string2!=""){
                static string cmd;
                cmd="UPDATE my_module_gpio_v2 set output_user_set_string='"+my_module_gpio_v2s[i].my_node->output_user_set_string2+"'";
                cmd=cmd+" where name_seri='"+my_module_gpio_v2s[i].my_node->name_node+"'";
                stmt->execute(cmd);
                my_module_gpio_v2s[i].my_node->output_user_set_string2="";
            }
            // update msg mission_normal
            if(my_module_gpio_v2s[i].my_node->mission_normal!=""){
                static string cmd;
                // update mission normal
                cmd="UPDATE my_module_gpio_v2 set mission_normal='"+my_module_gpio_v2s[i].my_node->mission_normal+"'";
                cmd=cmd+" where name_seri='"+my_module_gpio_v2s[i].my_node->name_node+"'";
                stmt->execute(cmd);
                // update mission normal backup
                cmd="UPDATE my_module_gpio_v2 set mission_normal_backup='"+my_module_gpio_v2s[i].my_node->mission_normal+"'";
                cmd=cmd+" where name_seri='"+my_module_gpio_v2s[i].my_node->name_node+"'";
                stmt->execute(cmd);
                //
                my_module_gpio_v2s[i].my_node->mission_normal="";
            }

        }catch (sql::SQLException &e) {
            cout << "# ERR: " << e.what();
            cout << " (MySQL error code: " << e.getErrorCode();
            cout << ", SQLState: " << e.getSQLState() << " )" << endl;
        }
        //
    }
}
void pub_output_user_status_string(string data){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub_output_user_status_string = n.advertise<std_msgs::String>("/output_user_status_string",100);
    static float creat_fun=0;
    static std_msgs::String msg;
        if(creat_fun==1){
                msg.data=data;
                pub_output_user_status_string.publish(msg);
        }else creat_fun=1;
}
void pub_input_user_status_string(string data){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub_input_user_status_string = n.advertise<std_msgs::String>("/input_user_status_string",100);
    static float creat_fun=0;
    static std_msgs::String msg;
        if(creat_fun==1){
                msg.data=data;
                pub_input_user_status_string.publish(msg);
        }else creat_fun=1;
}
void pub_output_user_set_string(string data){
    static ros::NodeHandle n,n1;
    static ros::Publisher pub_output_user_status_string = n.advertise<std_msgs::String>("/output_user_set_string",100);
    static float creat_fun=0;
    static std_msgs::String msg;
        if(creat_fun==1){
                msg.data=data;
                pub_output_user_status_string.publish(msg);
        }else creat_fun=1;
}
