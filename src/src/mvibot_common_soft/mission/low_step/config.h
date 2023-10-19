using namespace std;
//
class config_{
    public:
        string data;
        int  num_tab;
        // 
        string footprint_padding="none";
		//
        string max_vel_x="none";
		string acc_lim_x="none";
        string max_vel_theta="none";
        string acc_lim_theta="none";
        //
        string inflation_radius="none";
        int status=0;
        void print(int n);
        void process_data();
        int action(int action);
        void reset(); 
}; 
void config_::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"~","=","~");
    for(int i=0;i<data_return.data1.size();i++){
        if(data_return.data1[i]=="footprint_padding")       footprint_padding=data_return.data2[i];
        if(data_return.data1[i]=="max_vel_x")               max_vel_x=data_return.data2[i];
        if(data_return.data1[i]=="acc_lim_x")               acc_lim_x=data_return.data2[i];
        if(data_return.data1[i]=="max_vel_theta")           max_vel_theta=data_return.data2[i];
        if(data_return.data1[i]=="acc_lim_theta")           acc_lim_theta=data_return.data2[i];
        if(data_return.data1[i]=="inflation_radius")        inflation_radius=data_return.data2[i];
    }
}
void config_::print(int n){
    num_tab=n;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"Data:"<<data<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"footprint_padding:"<<footprint_padding<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"max_vel_x:"<<max_vel_x<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"acc_lim_x:"<<acc_lim_x<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"max_vel_theta:"<<max_vel_theta<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"acc_lim_theta:"<<acc_lim_theta<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"inflation_radius:"<<inflation_radius<<endl;
}
int config_::action(int action){
    static int value_return;
    if(action==Active_){
        static string config_set,config_return;
        value_return=Finish_;
        // footprint padding
        if(footprint_padding!="none"){
            if(stof_f(footprint_padding)>=0){
                // set local
                config_return=set_get_param("/"+mvibot_seri+"/move_base_flex/local_costmap/set_parameters","footprint_padding","double",footprint_padding);
                if(fabs(stof_f(footprint_padding)-stof_f(config_return))>0.001) value_return=Active_;
                // set global
                config_return=set_get_param("/"+mvibot_seri+"/move_base_flex/global_costmap/set_parameters","footprint_padding","double",footprint_padding);
                if(fabs(stof_f(footprint_padding)-stof_f(config_return))>0.001) value_return=Active_;
            }
            // 
        }
        // max_vel_x
        if(max_vel_x!="none"){
            if(stof_f(max_vel_x)>0){
                config_return=set_get_param("/"+mvibot_seri+"/move_base_flex/DWAPlannerROS/set_parameters","max_vel_x","double",max_vel_x);              
                if(fabs(stof_f(max_vel_x)-stof_f(config_return))>0.001) value_return=Active_;
            }
        }
        // acc_lim_x
        if(acc_lim_x!="none"){
            if(stof_f(acc_lim_x)>0){
                config_return=set_get_param("/"+mvibot_seri+"/move_base_flex/DWAPlannerROS/set_parameters","acc_lim_x","double",acc_lim_x);              
                if(fabs(stof_f(acc_lim_x)-stof_f(config_return))>0.001) value_return=Active_;
            }
        }
        // max_vel_theta
        if(max_vel_theta!="none"){
            if(stof_f(max_vel_theta)>0){
                config_return=set_get_param("/"+mvibot_seri+"/move_base_flex/DWAPlannerROS/set_parameters","max_vel_theta","double",max_vel_theta);              
                if(fabs(stof_f(max_vel_theta)-stof_f(config_return))>0.001) value_return=Active_;
            }
        }
        // acc_lim_theta
        if(acc_lim_theta!="none"){
            if(stof_f(acc_lim_theta)>0){
                config_return=set_get_param("/"+mvibot_seri+"/move_base_flex/DWAPlannerROS/set_parameters","acc_lim_theta","double",acc_lim_theta);              
                if(fabs(stof_f(acc_lim_theta)-stof_f(config_return))>0.001) value_return=Active_;
            }
        }
        // inflation_radius
        if(inflation_radius!="none"){
            if(stof_f(inflation_radius)>=0){
                // set local
                config_return=set_get_param("/"+mvibot_seri+"/move_base_flex/local_costmap/set_parameters","inflation_radius","double",inflation_radius);
                if(fabs(stof_f(inflation_radius)-stof_f(config_return))>0.001) value_return=Active_;
                // set global
                config_return=set_get_param("/"+mvibot_seri+"/move_base_flex/global_costmap/set_parameters","inflation_radius","double",inflation_radius);
                if(fabs(stof_f(inflation_radius)-stof_f(config_return))>0.001) value_return=Active_;
            }
            // 
        }
        return value_return;
    }else{
        value_return=action;
    }
    return value_return;
}
void config_::reset(){
    
}