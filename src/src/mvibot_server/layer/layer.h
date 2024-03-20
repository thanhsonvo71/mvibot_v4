#include "../mvibot_server_init.h"
using namespace  std;
void layer_process(){
    for(int i=0;i<my_layers.size();i++){
        my_layers[i].is_have=0;
    }
    try{
        free_res();
        res=stmt->executeQuery("SELECT * from `layer_emulator`");
        while (res->next()) {
            static int is_have;
            is_have=0;
            static string name_layer;
            name_layer=res->getString("name_layer");
            for(int i=0;i<my_layers.size();i++){
                if(name_layer == my_layers[i].name_layer){
                    is_have=1;
                    my_layers[i].is_have=1;
                }
            }
            if(is_have==0){
                    cout<<"insert_layer:"<<endl;
                    my_layers.resize(my_layers.size()+1);
                    my_layers[my_layers.size()-1].name_map_active=res->getString("name_map_active");
                    my_layers[my_layers.size()-1].name_layer=res->getString("name_layer");
                    my_layers[my_layers.size()-1].type_layer=res->getString("type_layer");
                    my_layers[my_layers.size()-1].heigth=atof(res->getString("height").c_str());
                    my_layers[my_layers.size()-1].width=atof(res->getString("width").c_str());
                    my_layers[my_layers.size()-1].xo=atof(res->getString("xo").c_str());
                    my_layers[my_layers.size()-1].yo=atof(res->getString("yo").c_str());
                    my_layers[my_layers.size()-1].yawo=atof(res->getString("yawo").c_str());
                    my_layers[my_layers.size()-1].is_have=1;
            }
        }
    }catch(sql::SQLException &e){
        cout << "# ERR: " << e.what();
        cout << " (MySQL error code: " << e.getErrorCode();
        cout << ", SQLState: " << e.getSQLState() << " )" << endl;
    }
}