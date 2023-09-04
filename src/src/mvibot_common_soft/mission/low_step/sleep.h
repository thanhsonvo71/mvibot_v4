using namespace std;
//
void sleep_::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"~","=","~");
    for(int i=0;i<data_return.data1.size();i++){
        if(data_return.data1[i]=="time_sleep")     time_sleep=stof(data_return.data2[i]);
    }
}
//
void sleep_::print(int n){
    num_tab=n;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"Data:"<<data<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"Time_sleep:"<<time_sleep<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"Time:"<<time<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"Status:"<<status<<endl;
}
//
int sleep_::action(int action){
    static int value_return;
    if(action==Active_){
        time+=ts_mission_step_scan;
        if(time>=time_sleep){
            status=Finish_;
            time=0;
        }
        else {
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Time_sleep_set:"<<time_sleep<<endl;
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Sleep:"<<time<<endl;
            status=Active_;
        }
    }else{
        time=0;
        status=action;
    }
    value_return=status;
    return value_return;
}
void sleep_::reset(){
    time=0;
    status=0;
}
//