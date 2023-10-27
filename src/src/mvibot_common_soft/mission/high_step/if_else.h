using namespace std;
//
void if_else_step::process_data(){
    static vector<string> my_detect;
    my_detect=detect_data_v1(data,'{','}');
    if(my_detect.size() % 2 ==0)
    for(int i=0;i<my_detect.size();i=i+2){
        if(my_detect[i]=="if_step"){
            if_step=new multiple_step_II;
            if_step->data=my_detect[i+1];
        }
        if(my_detect[i]=="else_step"){
            else_step=new multiple_step_II;
            else_step->data=my_detect[i+1];
        }
        if(my_detect[i]=="condition_step"){
            condition_step=new multiple_step_II;
            condition_step->data=my_detect[i+1];
        }
    }
    if(if_step!=nullptr) if_step->process_data();
    if(else_step!=nullptr) else_step->process_data();
    if(condition_step!=nullptr) condition_step->process_data();
}
void if_else_step::print(int n){
    num_tab=n;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"condition_step:"<<endl;
    if(condition_step!=nullptr) condition_step->print(n+1);
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"if_step:"<<endl;
    if(if_step!=nullptr) if_step->print(n+1);
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"else_step:"<<endl;
    if(else_step!=nullptr) else_step->print(n+1);
    //
}
//
int if_else_step::action(int action){
    int value_return;
    static int res;
    if(status==0){
        if(action!=Cancel_) {
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Action: Condition"<<endl;
        }
        res=condition_step->action(action);
        //
        if(res==Finish_ | res==True_) status=1;
        if(res==Error_  | res==False_) status=2;
        value_return=Active_;
        if(res==Break_){
            value_return=Finish_;
            status=0;
        }
    }
    else if(status==1){
        if(action!=Cancel_){
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Action: True condition"<<endl;
        }
        res=if_step->action(action);
        if(res==Finish_) {
            status=0;
        }
        value_return=res;
    }
    else if(status==2){
        if(action!=Cancel_){
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Action: False condition"<<endl;
        }
        if(else_step!=nullptr)
        res=else_step->action(action);
        else res==Finish_;
        if(res==Finish_) {
            status=0;
        }
        value_return=res;
    }
    return value_return;
}
void if_else_step::reset(){
    status=0;
    //
    if(condition_step!=nullptr) condition_step->reset();
    if(if_step!=nullptr)        if_step->reset();
    if(else_step!=nullptr)      else_step->reset();
}
int if_else_step::set_id(int n){
    static int value_return;
    value_return=condition_step->set_id(n);
    value_return=if_step->set_id(value_return);
    value_return=else_step->set_id(value_return);
    return value_return;
}