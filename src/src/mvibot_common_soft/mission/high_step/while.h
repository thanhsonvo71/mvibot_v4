using namespace std;
//
class while_step{
    public:
        string data;
        void process_data();
        void print(int n);
        int action(int action);
        void reset();
        int status=0;
        int num_tab;
        int set_id(int n);
        //
        multiple_step_II * condition_step=nullptr;
        multiple_step_II * do_step=nullptr;
};
void while_step::process_data(){
    static vector<string> my_detect;
    my_detect=detect_data_v1(data,'{','}');
    if(my_detect.size() % 2 ==0)
    for(int i=0;i<my_detect.size();i=i+2){
        if(my_detect[i]=="do_step"){
            do_step=new multiple_step_II;
            do_step->data=my_detect[i+1];
            do_step->process_data();
        }
        if(my_detect[i]=="condition_step"){
            condition_step=new multiple_step_II;
            condition_step->data=my_detect[i+1];
            condition_step->process_data();
        }
    }
}
void while_step::print(int n){
    num_tab=n;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"condition_step:"<<endl;
    if(condition_step!=nullptr) condition_step->print(n+1);
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"do_step:"<<endl;
    if(do_step!=nullptr) do_step->print(n+1);
    //
}
//
int while_step::action(int action){
    static int value_return;
    static int res;
    if(action==Active_){
        if(status==0){
            for(int j=0;j<num_tab;j++) cout<<"\t";
            cout<<"Action_condition:"<<endl;
            res=condition_step->action(Active_);
            //
            if(res == False_ | res==Error_){
                status=2;
                condition_step->reset();
                for(int j=0;j<num_tab;j++) cout<<"\t";
                cout<<"False_condition-> Break loop!"<<endl;
            }
            else if(res == True_ | res==Finish_){
                status=1;
                condition_step->reset();
                for(int j=0;j<num_tab;j++) cout<<"\t";
                cout<<"True_condition-> Continue loop"<<endl;
            }
            value_return=Active_;
        }
        if(status==1){
            for(int j=0;j<num_tab;j++) cout<<"\t";
            cout<<"Loop do:"<<endl;
            //
            res=do_step->action(Active_);
            if(res==Break_){
                value_return=Finish_;
            }else{
                //
                if(res!=Active_) do_step->reset();
                //
                if((res==Finish_ | res==True_ | res==False_)){
                    status=0;
                }
                else{
                    if(res==Error_) value_return=Error_;
                    else value_return=Active_;
                }
            }
        }
        if(status==2){
            value_return=Finish_;
        }
    }else{
        status=0;
        condition_step->action(Cancel_);
        do_step->action(Cancel_);
        value_return=Cancel_;
    }
    return value_return;
}
void while_step::reset(){
    status=0;
    //
    condition_step->reset();
    do_step->reset();
    //
}
int while_step::set_id(int n){
    static int value_return;
    value_return=condition_step->set_id(n);
    value_return=do_step->set_id(value_return);
    return value_return;
}