using namespace std;
//
class multiple_mission{
    public:
        string data="";
        vector<mission*> multiple_mission;
        void process_data();
        int action(int action);
        int num_mission_action=-1;
        void print(int n);
        void reset();
        void delete_free();
        string get_infor();
};
void multiple_mission::process_data(){
    static string_Iv2 data1;
    //
    data1.detect(data,"&","","@");
    //
    delete_free();
    multiple_mission.clear();
    multiple_mission.resize(data1.data1.size());
    //
    for(int i=0;i<data1.data1.size();i++){
        multiple_mission[i]=new mission;
        multiple_mission[i]->data=data1.data1[i];
        multiple_mission[i]->process_data();
    }
    //
}
void multiple_mission::print(int n){
    for(int i=0;i<multiple_mission.size();i++){
        multiple_mission[i]->print(n);
    }
}
int multiple_mission::action(int action){
   int value_return;
    if(action==Cancel_){
        static int res;
        if(num_mission_action==-1){
            for(int i=0;i<multiple_mission.size();i++){
                cout<<"Scan Mission "<<multiple_mission[i]->name_mission<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
                res=multiple_mission[i]->action(Cancel_);
                if(res==Wake_up_){
                    cout<<BOLDYELLOW<<"Mission "<<multiple_mission[i]->name_mission<<" is wake_up ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<endl;
                    num_mission_action=i;
                    value_return=Wake_up_;
                    break;
                }
            }
        }else{
            cout<<"Scan Mission "<<multiple_mission[num_mission_action]->name_mission<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
            res=multiple_mission[num_mission_action]->action(Cancel_);
            //
            if(res==Wake_up_){
                cout<<BOLDYELLOW<<"Mission "<<multiple_mission[num_mission_action]->name_mission<<" is wake_up ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<endl;
                value_return=Wake_up_;
            }
            //
            if(res==Continue_){
                cout<<BOLDYELLOW<<"Mission "<<multiple_mission[num_mission_action]->name_mission<<" is continue ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<endl;
                value_return=Continue_;
            }
        }
    }else if(action==Active_){
        if(num_mission_action!=-1 & num_mission_action<multiple_mission.size()){
            // cout<<BOLDYELLOW<<"Action Mission "<<num_mission_action<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<RESET<<endl;
            static int res;
            res=multiple_mission[num_mission_action]->action(Active_);
            if(res==Finish_){
                value_return=Finish_;
                num_mission_action=-1;
            }
            value_return=res;
        }else{
            value_return=Finish_;
            num_mission_action=-1;
        }
    }else if(action==Error_){
        for(int i=0;i<multiple_mission.size();i++){
            multiple_mission[i]->action(Cancel_);
        } 
    }
    return value_return;
}
void multiple_mission::reset(){
    num_mission_action=-1;
    for(int i=0;i<multiple_mission.size();i++){
        multiple_mission[i]->reset();
    }
}
void multiple_mission::delete_free(){
    for(int i=0;i<multiple_mission.size();i++){
        multiple_mission[i]->delete_free();
        delete multiple_mission[i];
    }
    multiple_mission.clear();
} 
string multiple_mission::get_infor(){
    static string value_return;
    value_return="";
    if(multiple_mission.size()>0){
        if(num_mission_action!=-1 & num_mission_action<multiple_mission.size()){
            value_return=multiple_mission[num_mission_action]->get_infor();
        }
    }
    return value_return;
}
