#include "../common/libary/libary_basic.h"
#include "../common/libary/libary_ros.h"
#include "../common/string_Iv2/string_Iv2.h"

#include "mysql_connection.h"
#include <mysql_driver.h>
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
using namespace std;
using namespace sql;
#if !defined(mvibot_server_define)
    // mysql var
    sql::mysql::MySQL_Driver *driver;
    Connection *con;
    sql::Statement *stmt;
    sql::ResultSet  *res;
    string user="mvibot",password="Mvibot@v1",host="tcp://127.0.0.1:3306",db="mvibot_database";
    // class table
    class colume_{
        public:
            string colume_name;
            string colume_type;
            int is_have=0;
    };
    class table_{
        public:
            string table_name;
            vector<colume_> table_colume;
            //
            void add_colume(string name, string type);
            void init_table();
    };
    void data_base_init();
    void table_init();
    // creat table
    table_ my_robot;
    table_ robot_status;
    table_ sensor_status;
    table_ battery_status;
    table_ battery_cell_status;
    table_ motor_right_status;
    table_ motor_left_status;
    table_ input_user_status;
    table_ output_user_status;
    table_ layer_emulator;
    table_ map_active;
    table_ robot_config_status;
    table_ my_module_gpio_v2;
    //
    class robot_information{
        public:
            //
            int update_database;
            float time_out=0;
            //
            string name_seri;
            string type;
            //
            string sensor_status;
            string robot_status;
            string battery_status;
            string battery_cell_status;
            string input_user_status;
            string output_user_status;
            string motor_left_status;
            string motor_right_status;
            string robot_config_status;
            //
            std::vector<string>     cmd_update_database();
            std::vector<string>     cmd_insert_database();
            string                  get_cmd_update_database(string data1_,string data2_);
            void                    update_status_robot(int n);
            void                    send_cmd_to_msyql(string cmd);
    };
    vector<robot_information> my_robots;
    float ts_my_robots;
    //
    class custom_layerv2{
        public:
            string name_layer;
            string type_layer;
            string name_map_active;
            float heigth,width;
            float xo,yo,yawo;
            int is_have=0;
            int is_pub=0;
    };
    vector<custom_layerv2> my_layers;
    void layer_process();
    int layer_change=1;
    // map var
    nav_msgs::OccupancyGrid my_map;
    nav_msgs::OccupancyGrid map_selector;
    string name_map_active="";
    int request_map_topic=0;
    //
    void free_res();
    void database_process();
    void get_robots_frist();
    // moudle gpio v2
    class node{
        public:
            std::string output_user_set_string;
            std::string output_user_status_string;
            std::string input_user_status_string;
            std::string name_node;
            ros::NodeHandle n;
            ros::Subscriber sub;
            void output_user_set(const std_msgs::String & msg){
                // cout<<msg.data<<endl;
                // cout<<name_node<<endl;
                output_user_set_string=name_node+"||"+msg.data;
            }
            void init(){
                sub = n.subscribe("/"+name_node+"/output_user_set", 1,&node::output_user_set,this);
            }
    };
    class module_gpio_v2_information{
        public:
            string name_seri;
            int    status;
            int    update_database;
            node*  my_node;
    };
    vector<module_gpio_v2_information> my_module_gpio_v2s;
    void get_module_gpio_v2_list();
    void update_module_gpio_v2();
    void pub_output_user_status_string(string data);
    void pub_input_user_status_string(string data);
    void pub_output_user_set_string(string data);
    #define mvibot_server_define 1
#endif