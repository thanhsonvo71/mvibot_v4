using namespace std;
//
void logic_and_step::process_data(){
    static vector<string> my_detect;
    my_detect=detect_data_v1(data,'{','}');
    if(my_detect.size() % 2 ==0)
    for(int i=0;i<my_detect.size();i=i+2){
        if(my_detect[i]=="logic_A"){
            logic_A=new multiple_step_II;
            logic_A->data=my_detect[i+1];
        }
        if(my_detect[i]=="logic_B"){
            logic_B=new multiple_step_II;
            logic_B->data=my_detect[i+1];
        }
    }
    if(logic_A!=nullptr) logic_A->process_data();
    if(logic_B!=nullptr) logic_B->process_data();
}
void logic_and_step::print(int n){
    num_tab=n;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"logic_A:"<<endl;
    if(logic_A!=nullptr) logic_A->print(n+1);
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"logic_B:"<<endl;
    if(logic_B!=nullptr) logic_B->print(n+1);
    //
}
int logic_and_step::action(int action){ 
    static int value_return;
    static int res;
    if(action==Active_){
        if(status_A==0){
            //
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Action: logic_A"<<endl;
            res=logic_A->action(Active_);
            if(res==Finish_ | res==Error_ | res==True_ | res==False_){
                status_A=res;
            }
        }else {
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Finish: logic_A"<<endl;
        }
        //
        if(status_B==0){
            //
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Action: logic_B"<<endl;
            res=logic_B->action(Active_);
            if(res==Finish_ | res==Error_ | res==True_ | res==False_){
                status_B=res;
            }
        }else{
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Finish: logic_B"<<endl;
        }
        //
        if(status_A!=0 & status_B!=0){
            //
            if((status_A==Finish_ | status_A==True_) & (status_B==Finish_ | status_B==True_)) {
                value_return=True_;
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Logic: True"<<endl;
            }
            else {
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Logic: False"<<endl;
                value_return=False_;
            }
        }else value_return=Active_;
    }else{
        status=0;
        status_A=0;
        status_B=0;
        logic_A->action(Cancel_);
        logic_B->action(Cancel_);
        value_return=action;
    }
    return value_return;
}
void logic_and_step::reset(){
    status=0;
    status_A=0;
    status_B=0;
    //
    logic_A->reset();
    logic_B->reset();
}
int logic_and_step::set_id(int n){
    static int value_return;
    value_return=logic_A->set_id(n);
    value_return=logic_B->set_id(value_return);
    return value_return;
}