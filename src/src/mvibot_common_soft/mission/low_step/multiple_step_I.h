using namespace std;
//
void multiple_step_I::process_data(){
    //
    n_step=0;
    action_step=0;
    static string_Iv2 data_return;
    data_return.detect(data,"(","",")");
    n_step=data_return.data1.size();
    my_step.resize(n_step);
    for(int i=0;i<n_step;i++){
        my_step[i].data=data_return.data1[i];
        my_step[i].process_data();
    }
}
void multiple_step_I::print(int n){
    num_tab=n;
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"N_step_I:"<<n_step<<endl;
    for(int i=0;i<my_step.size();i++){
        my_step[i].print(n+1);
    }
}
int multiple_step_I::action(int action){
    static int value_return;
    if(action_step<n_step){
        static int res;
        if(action==Active_) {
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Action step_I: "<<action_step<<endl;
        }
        res=my_step[action_step].action(action);
        if(res==Break_) {
            action_step=0;
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Have to break!"<<endl;
            value_return=Break_;
        }
        else{
            if(res==Finish_ | res==True_ | res==False_ ) action_step++;
            //
            if(res==Error_) value_return=Error_;
            else{
                // value_return=Active_;
                if(res==True_ | res==False_ ){
                    value_return=res;
                    action_step=0;
                }else value_return=Active_;
            }
        }
        //
    }else {
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"Finish step_I"<<endl;
        value_return=Finish_;
        action_step=0;
    }
    return value_return;
}
void multiple_step_I::reset(){
    action_step=0;
    for(int i=0;i<my_step.size();i++) my_step[i].reset();
}
int multiple_step_I::set_id(int n){
    static int res;
    res=n;
    for(int i=0;i<my_step.size();i++){
        res=my_step[i].set_id(res);
    }
    return res;
}