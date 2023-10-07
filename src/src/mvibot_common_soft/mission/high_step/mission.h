using namespace std;
//
void mission::process_data(){
    static string_Iv2 data1;
    //
    data1.detect(data,"","*","");
    my_multiple_step_II= new multiple_step_II;
    my_configuration= new multiple_step_I;
    if(data1.data1.size()==1){
        my_multiple_step_II->data=data;
        my_multiple_step_II->process_data();
        my_multiple_step_II->set_id(0);
        my_configuration->process_data();
    }else if(data1.data1.size()==2 & data1.data1[1]!="" ){
        //
        static string_Iv2 data2;
        data2.detect(data1.data1[0],"/",">","/");
        for(int i=0;i<data2.data1.size();i++){
            if(data2.data1[i]=="name_mission") name_mission=data2.data2[i];
            if(data2.data1[i]=="id_mission") id_mission=stoi_f(data2.data2[i]);
            if(data2.data1[i]=="data_configuration") {
                my_configuration->data=data2.data2[i];
                my_configuration->process_data();
            }
        }
        //
        my_multiple_step_II->data=data1.data1[1];
        my_multiple_step_II->process_data();
        my_multiple_step_II->set_id(0);
    }
    //
}
void mission::print(int n){
    cout<<BOLDYELLOW<<"Mission name: "<<name_mission<<"****************************************************************************************"<<RESET<<endl;
    cout<<BOLDGREEN<<"Mission id: "<<id_mission<<RESET<<endl;
    cout<<BOLDGREEN<<"Mission total step: "<<my_multiple_step_II->num_small_step_action<<" steps"<<RESET<<endl;
    cout<<BOLDGREEN<<"Mission config:"<<RESET<<endl;
    my_configuration->print(n+1);
    cout<<BOLDGREEN<<"Mission action:"<<RESET<<endl;
    if(my_multiple_step_II->data!="")my_multiple_step_II->print(n+1);
    cout<<BOLDYELLOW<<"************************************************************************************************************"<<RESET<<endl;
}
int mission::action(int action){
    //
    int value_return;
    //
    if(action==Cancel_){
        static int wake_up_true;
        wake_up_true=0;
        for(int i=0;i<my_configuration->my_step.size();i++){
            if(my_configuration->my_step[i].name_step=="wake_up"){
                if(my_configuration->my_step[i].mode_step=="gpio" | my_configuration->my_step[i].mode_step=="gpio_module"){
                    static int res;
                    res=my_configuration->my_step[i].action(Active_);
                    if(res==Finish_){
                        wake_up_true=1;
                        value_return=Wake_up_;
                        break;
                    }
                }
            }
            if(my_configuration->my_step[i].name_step=="continue"){
                if(my_configuration->my_step[i].mode_step=="gpio" | my_configuration->my_step[i].mode_step=="gpio_module"){
                    static int res;
                    res=my_configuration->my_step[i].action(Active_);
                    if(res==Finish_){
                        wake_up_true=1;
                        value_return=Continue_;
                        break;
                    }
                }
            }
        }
        if(wake_up_true==0){
            my_multiple_step_II->action(Cancel_);
        } else {
            cout<<BOLDGREEN<<"Start active because the config action start is True"<<RESET<<endl;
        }
    }else if(action==Active_){
        static int stop_true;
        stop_true=0;
        for(int i=0;i<my_configuration->my_step.size();i++){
            if(my_configuration->my_step[i].name_step=="stop"){
                if(my_configuration->my_step[i].mode_step=="gpio" | my_configuration->my_step[i].mode_step=="gpio_module"){
                    static int res;
                    res=my_configuration->my_step[i].action(Active_);
                    if(res==Finish_ & action_mission==Active_) {
                        value_return=Stop_;
                        stop_true=1;
                        break;
                    }
                }
            }
        }
        if(stop_true==0){
            // value_return=my_multiple_step_II->action(Active_);
            if(my_multiple_step_II->data!=""){
                if(action==Active_){
                    cout<<"---------------------------------------------------------------------------------------------------------------------------------------------------------"<<endl;
                    cout<<"Action Mission-> "<<step_action_information<<"/"<<my_multiple_step_II->num_small_step_action<<" steps"<<endl;
                }
                value_return=my_multiple_step_II->action(action);
                if(value_return==Finish_ | value_return==True_ | value_return==False_ | value_return==Break_){
                    cout<<BOLDMAGENTA;
                    reset();
                    //my_vars_local.reset();
                    //
                    infor_action_step="";
                    step_action_information="0";
                    //
                    cout<<"Finish Mission"<<endl;
                    cout<<RESET;
                }
                //
                if(action==Active_){
                    //my_vars_local.print();
                    cout<<BOLDYELLOW<<"Name mision:"<<name_mission<<endl;
                    cout<<"id:"<<id_mission<<endl;
                    cout<<"infor_step:"<<infor_action_step<<endl;
                    if(value_return==Error_) cout<<"Mission Error at step :"<<infor_action_step<<endl;
                    cout<<step_action_information<<"/"<<my_multiple_step_II->num_small_step_action<<" steps"<<RESET<<endl;
                    cout<<"---------------------------------------------------------------------------------------------------------------------------------------------------------"<<endl;
                }
            }
            else value_return=Finish_;
        }else{
            cout<<BOLDRED<<"Stop active because the config action stop is True"<<RESET<<endl;
            value_return=Stop_;
        }
        //
    }
    return value_return;
}
void mission::reset(){
    //
    cout<<"Reset Mission"<<endl;
    my_configuration->reset();
    my_multiple_step_II->reset();
    
    //
}
void mission::delete_free(){
   if(my_configuration!=nullptr){
        delete my_configuration;
        my_configuration=nullptr;
   }
   if(my_multiple_step_II!=nullptr){
        my_multiple_step_II->delete_free();
        delete my_multiple_step_II;
        my_multiple_step_II=nullptr;
   }
}
string mission::get_infor(){
    static string value_return;
    value_return="";
    value_return+="(name_mission:"+name_mission+")";
    value_return+="(step_action_information:"+step_action_information+")";
    value_return+="(total_step:"+to_string(my_multiple_step_II->num_small_step_action)+")";
    value_return+="(infor_action_step:"+infor_action_step+")";
    return value_return;
}