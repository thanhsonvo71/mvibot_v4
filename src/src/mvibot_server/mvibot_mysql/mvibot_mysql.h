#include "../mvibot_server_init.h"
using namespace std;
void data_base_init(){
    // creat connect to data base
    static int init_finish;
    init_finish=0;
    while(init_finish==0){
        try{
            driver = sql::mysql::get_mysql_driver_instance();
            con = driver->connect(host, user, password);
            if(con->isValid()){
                cout <<"Finish connect!"<<endl;
                // creat statement
                stmt = con->createStatement();
                cout<<"Creat database"<<endl;
                stmt->execute("CREATE DATABASE IF NOT EXISTS "+db);
                cout<<"Use database"<<endl;
                stmt->execute("USE "+db);
                //
                init_finish=1;
            }
        }catch (sql::SQLException &e) {
            cout << "# ERR: " << e.what();
            cout << " (MySQL error code: " << e.getErrorCode();
            cout << ", SQLState: " << e.getSQLState() << " )" << endl;
            cout << "reconnect....."<<endl;
            init_finish=0;
            sleep(1);
        }
        
    }
    //
}
void table_init(){
    //
    my_robot.table_name="my_robot";
    my_robot.add_colume("id","bigint");
    my_robot.add_colume("name_seri","varchar(255)");
    my_robot.add_colume("type","varchar(255)");
    my_robot.init_table();
    //
    robot_status.table_name="robot_status";
    robot_status.add_colume("id","bigint");
    robot_status.add_colume("name_seri","varchar(255)");
    robot_status.add_colume("status","varchar(255)");
    robot_status.add_colume("mode","varchar(255)");
    robot_status.add_colume("mode_status","varchar(255)");
    robot_status.add_colume("ip_node","varchar(255)");
    robot_status.add_colume("ip_master","varchar(255)");
    robot_status.add_colume("type_connect","varchar(255)");
    robot_status.init_table();
    //
    sensor_status.table_name="sensor_status";
    sensor_status.add_colume("id","bigint");
    sensor_status.add_colume("name_seri","varchar(255)");
    sensor_status.add_colume("uart","int");
    sensor_status.add_colume("radar1","int");
    sensor_status.add_colume("radar2","int");
    sensor_status.add_colume("camera1","int");
    sensor_status.add_colume("camera2","int");
    sensor_status.add_colume("battery","int");
    sensor_status.add_colume("battery_small","int");
    sensor_status.init_table();
    //
    battery_status.table_name="battery_status";
    battery_status.add_colume("id","bigint");
    battery_status.add_colume("name_seri","varchar(255)");
    battery_status.add_colume("soc","int");
    battery_status.add_colume("vol","float(4,2)");
    battery_status.add_colume("cycle","int");
    battery_status.add_colume("capacity_now","float(6,3)");
    battery_status.add_colume("capacity_max","float(6,3)");    
    battery_status.add_colume("charge","int");    
    battery_status.add_colume("current","float(5,2)");    
    battery_status.add_colume("num_cell","int");    
    battery_status.add_colume("temperature","float(3,1)");    
    battery_status.init_table();
    //
    battery_cell_status.table_name="battery_cell_status";
    battery_cell_status.add_colume("id","bigint");
    battery_cell_status.add_colume("name_seri","varchar(255)");
    for(int i=0;i<8;i++){
    	    battery_cell_status.add_colume("cell"+to_string(i+1),"float(2,1)");
    }
    battery_cell_status.init_table();
    //
    motor_right_status.table_name="motor_right_status";
    motor_right_status.add_colume("id","bigint");
    motor_right_status.add_colume("name_seri","varchar(255)");
    motor_right_status.add_colume("live","int");
    motor_right_status.add_colume("error","int");
    motor_right_status.add_colume("enable","int");
    motor_right_status.add_colume("brake","int");
    motor_right_status.init_table();
    //
    motor_left_status.table_name="motor_left_status";
    motor_left_status.add_colume("id","bigint");
    motor_left_status.add_colume("name_seri","varchar(255)");
    motor_left_status.add_colume("live","int");
    motor_left_status.add_colume("error","int");
    motor_left_status.add_colume("enable","int");
    motor_left_status.add_colume("brake","int");
    motor_left_status.init_table();
    //
    input_user_status.table_name="input_user_status";
    input_user_status.add_colume("id","bigint");
    input_user_status.add_colume("name_seri","varchar(255)");
    for(int i=0;i<30;i++){
    	input_user_status.add_colume("in"+to_string(i+1),"int");
    }
    input_user_status.init_table();
    //
    output_user_status.table_name="output_user_status";
    output_user_status.add_colume("id","bigint");
    output_user_status.add_colume("name_seri","varchar(255)");
    for(int i=0;i<30;i++){
    	output_user_status.add_colume("out"+to_string(i+1),"int");
    }
    output_user_status.init_table();
    //
    layer_emulator.table_name="layer_emulator";
    layer_emulator.add_colume("id","bigint");
    layer_emulator.add_colume("name_map_active","varchar(255)");   
    layer_emulator.add_colume("name_layer","varchar(255)");     
    layer_emulator.add_colume("type_layer","varchar(255)");         
    layer_emulator.add_colume("height","float(15,3)");         
    layer_emulator.add_colume("width","float(15,3)");   
    layer_emulator.add_colume("xo","float(15,3)");   
    layer_emulator.add_colume("yo","float(15,3)");   
    layer_emulator.add_colume("yawo","float(15,3)");   
    layer_emulator.init_table();
    //
    map_active.table_name="map_active";
    map_active.add_colume("name_map_active","varchar(255)");     
    map_active.init_table();
    //
    robot_config_status.table_name="robot_config_status";
    robot_config_status.add_colume("id","bigint");
    robot_config_status.add_colume("name_seri","varchar(255)");   
    robot_config_status.add_colume("robot_R","varchar(255)");     
    robot_config_status.add_colume("robot_L","varchar(255)");         
    robot_config_status.add_colume("robot_gear","varchar(255)");     
    robot_config_status.add_colume("robot_ax","varchar(255)");         
    robot_config_status.add_colume("robot_aw","varchar(255)");     
    robot_config_status.add_colume("robot_vmax","varchar(255)");         
    robot_config_status.add_colume("robot_wmax","varchar(255)");     
    robot_config_status.add_colume("robot_volume","varchar(255)");         
    robot_config_status.add_colume("robot_low_battery","varchar(255)");     
    robot_config_status.add_colume("robot_type_connect","varchar(255)");  
    robot_config_status.add_colume("serial_camera1","varchar(255)");     
    robot_config_status.add_colume("serial_camera2","varchar(255)");      
    robot_config_status.add_colume("lan_type","varchar(255)");     
    robot_config_status.add_colume("lan_ipv4","varchar(255)");         
    robot_config_status.add_colume("lan_ipv4_gateway","varchar(255)");     
    robot_config_status.add_colume("lan_ipv4_dns","varchar(255)");      
    robot_config_status.add_colume("wifi_type","varchar(255)");     
    robot_config_status.add_colume("wifi_ssid","varchar(255)");         
    robot_config_status.add_colume("wifi_password","varchar(255)");     
    robot_config_status.add_colume("wifi_ipv4","varchar(255)");   
    robot_config_status.add_colume("wifi_ipv4_gateway","varchar(255)");     
    robot_config_status.add_colume("wifi_ipv4_dns","varchar(255)");   
    robot_config_status.add_colume("mode","varchar(255)");     
    robot_config_status.add_colume("ip_master","varchar(255)");         
    robot_config_status.add_colume("is_master","varchar(255)");     
    robot_config_status.add_colume("ip_node","varchar(255)");      
    robot_config_status.init_table();
    //
    my_module_gpio_v2.table_name="my_module_gpio_v2";
    my_module_gpio_v2.add_colume("id","bigint");      
    my_module_gpio_v2.add_colume("name_seri","varchar(255)");
    my_module_gpio_v2.add_colume("status","int");
    my_module_gpio_v2.add_colume("input_status_string","varchar(255)");
    my_module_gpio_v2.add_colume("output_status_string","varchar(255)");
    my_module_gpio_v2.add_colume("output_user_set_string","varchar(255)");
    my_module_gpio_v2.add_colume("mission_normal","TEXT");
    my_module_gpio_v2.add_colume("battery","varchar(255)");
    my_module_gpio_v2.add_colume("output_user_set_string_fesp","varchar(255)");
    my_module_gpio_v2.init_table();
}
void table_:: add_colume(string name, string type){
    table_colume.resize(table_colume.size()+1);
    table_colume[table_colume.size()-1].colume_name=name;
    table_colume[table_colume.size()-1].colume_type=type;
}
void table_::init_table(){
    static int init_finish;
    init_finish=0;
    while(init_finish==0){
        try{
            cout<<"Creat table "<<table_name<<endl;
            static string cmd;
            cmd="CREATE TABLE IF NOT EXISTS "+table_name+""+" (id int)"; //, PRIMARY KEY (name_seri)
            stmt->execute(cmd);
            //
            for(int i=0;i<table_colume.size();i++){
                try{    
                    stmt->execute("SELECT `"+table_colume[i].colume_name+"`"+"from `"+table_name+"`");
                    stmt->getResultSet();
                }catch(sql::SQLException &e){
                    cout<<"add colume:"<<table_colume[i].colume_name<<endl;
                    stmt->execute("ALTER TABLE `"+table_name+"` ADD COLUMN `"+table_colume[i].colume_name+"` "+table_colume[i].colume_type);
                }
            }
            //
            init_finish=1;
        }catch (sql::SQLException &e) {
            cout << "# ERR: " << e.what();
            cout << " (MySQL error code: " << e.getErrorCode();
            cout << ", SQLState: " << e.getSQLState() << " )" << endl;
            cout << "reconnect....."<<endl;
            init_finish=0;
        }
    }
}
void database_process(){
    //
    get_module_gpio_v2_list();
    update_module_gpio_v2();
    //
    for(int i=0;i<my_robots.size();i++){
        // update new robot into database
        if(my_robots[i].update_database==0){
            cout<<"Start Add new robot:"<<my_robots[i].name_seri<<endl;
            static vector<string> data;
            data=my_robots[i].cmd_insert_database();
            for(int j=0;j<data.size();j++){
                try{
                    stmt->execute(data[j]);
                }catch (sql::SQLException &e) {
                    cout << "# ERR: " << e.what();
                    cout << " (MySQL error code: " << e.getErrorCode();
                    cout << ", SQLState: " << e.getSQLState() << " )" << endl;
                }
            }
            my_robots[i].update_database=1;
        }else{
            my_robots[i].time_out+=ts_my_robots;
            if(my_robots[i].time_out>=10) my_robots[i].time_out=10.0;
            cout<<my_robots[i].name_seri<<"|"<<my_robots[i].time_out<<endl;
            //
            if(my_robots[i].time_out>=3) my_robots[i].update_status_robot(0);
            else my_robots[i].update_status_robot(1);
            //
            my_robots[i].cmd_update_database();
        }
    }
}
void free_res(){
    if(res!=nullptr){
        delete res;
        res=nullptr;
    }
}