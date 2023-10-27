using namespace std;
//
void try_catch_step::process_data(){
    static vector<string> my_detect;
    my_detect=detect_data_v1(data,'{','}');
    if(my_detect.size() % 2 ==0)
    for(int i=0;i<my_detect.size();i=i+2){
        if(my_detect[i]=="try_step"){
            try_step=new multiple_step_II;
            try_step->data=my_detect[i+1];

        }
        if(my_detect[i]=="catch_step"){
            catch_step=new multiple_step_II;
            catch_step->data=my_detect[i+1];
        }
    }
    if(try_step!=nullptr) try_step->process_data();
    if(catch_step!=nullptr) catch_step->process_data();
}
void try_catch_step::print(int n){
    num_tab=n;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"try_step:"<<endl;
    if(try_step!=nullptr) try_step->print(n+1);
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"catch_step:"<<endl;
    if(catch_step!=nullptr) catch_step->print(n+1);
    //
}
//
int try_catch_step::action(int action){
    static int value_return;
    static int res;
    if(status==0){
        if(action!=Cancel_){
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Action: try"<<endl;
        }
        res=try_step->action(action);
        if(res==Error_){
            status=1;
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Error change to catch action"<<endl;
            value_return=Active_;
        }else{
            status=0;
            value_return=res; 
        }
    }
    else if(status==1){
        if(action!=Cancel_){
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Action: catch"<<endl;
        }
        res=catch_step->action(action);
        if(res==Finish_) {
            status=0;
        }
        value_return=res;
    }
    return value_return;
}
void try_catch_step::reset(){
    status=0;
    //
    if(try_step!=nullptr)       try_step->reset();
    if(catch_step!=nullptr)     catch_step->reset();
}
int try_catch_step::set_id(int n){
    static int value_return;
    value_return=try_step->set_id(n);
    value_return=catch_step->set_id(value_return);
    return value_return;
}