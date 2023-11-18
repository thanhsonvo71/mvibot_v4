#include "../mvibot_server_init.h"
using namespace std;
string robot_information::get_cmd_update_database(string data1_,string data2_){
    static string string_return;
    //
    static string_Iv2 data;
    static string string_cmd_mysql;
    data.detect(data2_,"","|","");
    string_cmd_mysql="";
    string_cmd_mysql=string_cmd_mysql+"UPDATE "+data1_+" SET ";
    for(int i=1;i<data.data1.size()-1;i++){
            static string_Iv2 data2;
            data2.detect(data.data1[i],"",":","");
            if(data2.data1[0]!="")
            string_cmd_mysql=string_cmd_mysql+data2.data1[0]+"="+"'"+data2.data1[1]+"', ";
    }
    static string_Iv2 data2;
    data2.detect(data.data1[data.data1.size()-1],"",":","");
    if(data2.data1[0]!="")
    string_cmd_mysql=string_cmd_mysql+data2.data1[0]+"="+"'"+data2.data1[1]+"' ";
    string_cmd_mysql=string_cmd_mysql+"WHERE name_seri = '"+data.data1[0]+"' ";
    string_return=string_cmd_mysql;
    return string_return;
}
std::vector<string>  robot_information::cmd_update_database(){
    static vector<string> string_return;
    //
    string_return.resize(0);
    //
    if(sensor_status!=""){
        string_return.resize(string_return.size()+1);
        string_return[string_return.size()-1]=get_cmd_update_database("sensor_status",sensor_status);
        send_cmd_to_msyql(string_return[string_return.size()-1]);
    }
    //
    if(robot_status!=""){
        string_return.resize(string_return.size()+1);
        string_return[string_return.size()-1]=get_cmd_update_database("robot_status",robot_status);
        send_cmd_to_msyql(string_return[string_return.size()-1]);

    }
    //
    if(battery_status!=""){
        string_return.resize(string_return.size()+1);
        string_return[string_return.size()-1]=get_cmd_update_database("battery_status",battery_status);
        send_cmd_to_msyql(string_return[string_return.size()-1]);

    }
    if(battery_small_status!=""){
        string_return.resize(string_return.size()+1);
        string_return[string_return.size()-1]=get_cmd_update_database("battery_small_status",battery_small_status);
        send_cmd_to_msyql(string_return[string_return.size()-1]);

    }
    //
    if(battery_cell_status!=""){
        string_return.resize(string_return.size()+1);
        string_return[string_return.size()-1]=get_cmd_update_database("battery_cell_status",battery_cell_status);
        send_cmd_to_msyql(string_return[string_return.size()-1]);
    }
    //
    if(motor_left_status!=""){
        string_return.resize(string_return.size()+1);
        string_return[string_return.size()-1]=get_cmd_update_database("motor_left_status",motor_left_status);
        send_cmd_to_msyql(string_return[string_return.size()-1]);
    }
    //
    if(motor_right_status!=""){
        string_return.resize(string_return.size()+1);
        string_return[string_return.size()-1]=get_cmd_update_database("motor_right_status",motor_right_status);
        send_cmd_to_msyql(string_return[string_return.size()-1]);
    }
    //
    if(input_user_status!=""){
        string_return.resize(string_return.size()+1);
        string_return[string_return.size()-1]=get_cmd_update_database("input_user_status",input_user_status);
        send_cmd_to_msyql(string_return[string_return.size()-1]);
    }
    //
    if(output_user_status!=""){
        string_return.resize(string_return.size()+1);
        string_return[string_return.size()-1]=get_cmd_update_database("output_user_status",output_user_status);
        send_cmd_to_msyql(string_return[string_return.size()-1]);
    }
    //
    if(robot_config_status!=""){
        string_return.resize(string_return.size()+1);
        string_return[string_return.size()-1]=get_cmd_update_database("robot_config_status",robot_config_status);
        send_cmd_to_msyql(string_return[string_return.size()-1]);
    }
    //

    if(node->update_mision==1){
        string_return.resize(string_return.size()+1);
        string_return[string_return.size()-1]="update `my_robot_backup_mission` set mission_normal_backup='"+node->mission_normal+"' where name_seri='"+name_seri+"'";
        send_cmd_to_msyql(string_return[string_return.size()-1]);
        node->update_mision=0;
    }
    return string_return;
}
std::vector<string>  robot_information::cmd_insert_database(){
    static vector<string> string_return;
    //
    string_return.resize(12);
    string_return[0]="INSERT INTO my_robot (name_seri,type)  VALUES('"+name_seri+"','"+type+"')";
    string_return[1]="INSERT INTO robot_status (name_seri)  VALUES('"+name_seri+"')";
    string_return[2]="INSERT INTO sensor_status (name_seri)  VALUES('"+name_seri+"')";
    string_return[3]="INSERT INTO battery_status (name_seri)  VALUES('"+name_seri+"')";
    string_return[4]="INSERT INTO battery_cell_status (name_seri)  VALUES('"+name_seri+"')";
    string_return[5]="INSERT INTO motor_left_status (name_seri)  VALUES('"+name_seri+"')";
    string_return[6]="INSERT INTO motor_right_status (name_seri)  VALUES('"+name_seri+"')";
    string_return[7]="INSERT INTO input_user_status (name_seri)  VALUES('"+name_seri+"')";
    string_return[8]="INSERT INTO output_user_status (name_seri)  VALUES('"+name_seri+"')";
    string_return[9]="INSERT INTO robot_config_status (name_seri)  VALUES('"+name_seri+"')";
    string_return[10]="INSERT INTO my_robot_backup_mission (name_seri)  VALUES('"+name_seri+"')";
    string_return[11]="INSERT INTO battery_small_status (name_seri)  VALUES('"+name_seri+"')";  
    return string_return;
}
void get_robots_frist(){
    //
    my_robots.resize(0);
    //
    try{
        // 
        free_res();
        res=stmt->executeQuery("SELECT * from `my_robot`");
        while (res->next()) {
            cout<<res->getString("name_seri")<<endl;
            my_robots.resize(my_robots.size()+1);
            my_robots[my_robots.size()-1].name_seri=res->getString("name_seri");
            my_robots[my_robots.size()-1].type=res->getString("type");
            my_robots[my_robots.size()-1].update_database=1;
            //
            my_robots[my_robots.size()-1].node= new node_v2_3;
            my_robots[my_robots.size()-1].node->name_seri=my_robots[my_robots.size()-1].name_seri;   
            my_robots[my_robots.size()-1].node->init();
        }
        // get mission backup
        free_res();
        res=stmt->executeQuery("SELECT * from `my_robot_backup_mission`");
        while (res->next()) {
            for(int i=0;i<my_robots.size();i++){
                if(res->getString("name_seri")==my_robots[i].name_seri){
                    my_robots[i].node->mission_normal=res->getString("mission_normal_backup");
                    break;
                }
            }
        }
      
    }catch(sql::SQLException &e){
        cout << "# ERR: " << e.what();
        cout << " (MySQL error code: " << e.getErrorCode();
        cout << ", SQLState: " << e.getSQLState() << " )" << endl;
    }
}
void robot_information::update_status_robot(int n){
    static string cmd;
    if(n==1) cmd="update `robot_status` set status='1' where name_seri='"+name_seri+"'";
    else cmd="update `robot_status` set status='0' where name_seri='"+name_seri+"'";
    try{
        stmt->execute(cmd);
    }catch (sql::SQLException &e) {
        cout << "# ERR: " << e.what();
        cout << " (MySQL error code: " << e.getErrorCode();
        cout << ", SQLState: " << e.getSQLState() << " )" << endl;
    }
}
void robot_information::send_cmd_to_msyql(string cmd){
    try{
        stmt->execute(cmd);
    }catch (sql::SQLException &e) {
        cout << "# ERR: " << e.what();
        cout << " (MySQL error code: " << e.getErrorCode();
        cout << ", SQLState: " << e.getSQLState() << " )" << endl;
    }
}
