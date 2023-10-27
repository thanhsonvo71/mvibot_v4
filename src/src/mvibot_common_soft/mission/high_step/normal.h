using namespace std;
//
void normal_step::process_data(){
    static vector<string> my_detect;
    my_detect=detect_data_v1(data,'{','}');
    if(my_detect.size() % 2 ==0)
    for(int i=0;i<my_detect.size();i=i+2){
        if(my_detect[i]=="normal_step"){
            normal_step=new multiple_step_II;
            normal_step->data=my_detect[i+1];
        }
    }
    if(normal_step!=nullptr) normal_step->process_data();
}
void normal_step::print(int n){
    num_tab=n;
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"normal_step:"<<endl;
    if(normal_step!=nullptr) normal_step->print(n+1);
}
int normal_step::action(int action){
    int value_return;
    static int res;
    //
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"Action: Normal"<<endl;
    res=normal_step->action(action);
    value_return=res;
    //
    return value_return;
    
}
void normal_step::reset(){
    if(normal_step!=nullptr)
    normal_step->reset();
}
int normal_step::set_id(int n){
    static int value_return;
    value_return=normal_step->set_id(n);
    return value_return;
}