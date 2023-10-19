using namespace std;
//
class footprint_{
    public:
        string data;
        int  num_tab;
		float x1;
		float y1;
		float x2;
		float y2;
        int status=0;
        void print(int n);
        void process_data();
        int action(int action);
        void reset(); 
}; 
void footprint_::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"~","=","~");
    for(int i=0;i<data_return.data1.size();i++){
        if(data_return.data1[i]=="x1")     x1=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="y1")     y1=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="x2")     x2=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="y2")     y2=stof_f(data_return.data2[i]);
    }
}
void footprint_::print(int n){
    num_tab=n;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"Data:"<<data<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"x1:"<<x1<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"y1:"<<y1<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"x2:"<<x2<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"y2:"<<y2<<endl;
}
int footprint_::action(int action){
    static int value_return;
    if(action==Active_){
        static string footprint_set;
        static string footprint_return;
        footprint_set="[";
        footprint_set=footprint_set+"["+to_string(x1)+","+to_string(y1)+"],";
        footprint_set=footprint_set+"["+to_string(-x1)+","+to_string(y1)+"],";
        footprint_set=footprint_set+"["+to_string(x2)+","+to_string(y2)+"],";
        footprint_set=footprint_set+"["+to_string(-x2)+","+to_string(y2)+"]]";
        cout<<"\t \t \t \t \t footprint set:"<<footprint_set<<endl;
        footprint_x1=x1; footprint_x2=x2; footprint_y1=y1, footprint_y2=y2;
        value_return=Finish_;
        // local
        footprint_return=set_get_param("/"+mvibot_seri+"/move_base_flex/local_costmap/set_parameters","footprint","string",footprint_set);
        cout<<"\t \t \t \t \t set footprint local costmap:"<<footprint_return<<endl;
        if(footprint_return!=footprint_set) value_return=Active_;
        // global
        footprint_return=set_get_param("/"+mvibot_seri+"/move_base_flex/global_costmap/set_parameters","footprint","string",footprint_set);
        cout<<"\t \t \t \t \t set footprint global costmap:"<<footprint_return<<endl;
        if(footprint_return!=footprint_set) value_return=Active_;
        //
    }else{
        value_return=action;
    }
    return value_return;
}
void footprint_::reset(){
    
}