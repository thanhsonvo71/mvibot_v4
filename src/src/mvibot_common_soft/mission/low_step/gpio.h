using namespace std;
//
class gpio_{
	public:
        string data;
        int num_tab;
        string in_pullup_string="";
        string in_pulldown_string="";
		// out put user
        int not_set_out=0;
		string out_set;
		string out_reset;
		// input user
		string in_on;
		string in_off;
		string in_pullup;
		string in_pulldown;
        int status;
        void print(int n);
        void process_data();
        int action(int action); 
        void reset();
};
void pub_gpio_set(string gpio_output_string){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/output_user_set", 1);
    static float creat_fun=0;
    static std_msgs::String msg;
    if(creat_fun==1)
    {
            msg.data=gpio_output_string;
            pub.publish(msg);
    } else creat_fun=1;
}
//
void gpio_::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"~","=","~");
    for(int i=0;i<data_return.data1.size();i++){
        if(data_return.data1[i]=="out_set")     out_set=data_return.data2[i];
        if(data_return.data1[i]=="out_reset")   out_reset=data_return.data2[i];
        if(data_return.data1[i]=="in_on")       in_on=data_return.data2[i];
        if(data_return.data1[i]=="in_off")      in_off=data_return.data2[i];
        if(data_return.data1[i]=="in_pullup")   in_pullup=data_return.data2[i];
        if(data_return.data1[i]=="in_pulldown") in_pulldown=data_return.data2[i];
        if(data_return.data1[i]=="not_set_out") not_set_out=stoi_f(data_return.data2[i]);  
    }
}
void gpio_::print(int n){
    num_tab=n;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"Data:"<<data<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"out_set:"<<out_set<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"out_reset:"<<out_reset<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"in_on:"<<in_on<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"in_off:"<<in_off<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"in_pullup:"<<in_pullup<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"in_pulldown:"<<in_pulldown<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"not_set_out:"<<not_set_out<<endl;
}
int gpio_::action(int action){
    static int value_return;
    static string in_pullup_string;
    static string in_pulldown_string;
    static vector<int> data2;
    static int is_have;
    if(action==Active_){
        gpio_msg="";
        // out_set
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"not_set_out:"<<not_set_out<<endl;
        static vector<int> data;
        static int output_set_true;
        output_set_true=1;
        if(out_set!="") {
            data=process_data2(out_set);
            for(int i=0;i<data.size();i++){
                if(data[i]<output_user_status.data.size()){
                    if(output_user_status.data[data[i]]!=1){
                        output_set_true=0;
                        gpio_msg=gpio_msg+"("+to_string(data[i])+"|1)";
                    }
                }else output_set_true=0;
            }
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"out_set:"<<out_set<<endl;
        }
        // out_reset
        if(out_reset!=""){
            data=process_data2(out_reset);
            for(int i=0;i<data.size();i++){
                if(data[i]<output_user_status.data.size()){
                    if(output_user_status.data[data[i]]!=0){
                        output_set_true=0;
                        gpio_msg=gpio_msg+"("+to_string(data[i])+"|0)";
                    }
                }else output_set_true=0;
            }
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"out_reset:"<<out_reset<<endl;
        }
        if(gpio_msg!="" & not_set_out==0) {
            pub_gpio_set(gpio_msg);
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"gpio_msg:"<<gpio_msg<<endl;
        }
        //
        if(output_set_true==1)
        value_return=Finish_;
        else value_return=Active_;
        // in_on
        static string in_on_string;
        in_on_string="";
        if(in_on!="") {
            data=process_data2(in_on);
            for(int i=0;i<data.size();i++){
                if(data[i]<input_user_status_1.data.size()){
                    if(input_user_status_1.data[data[i]]!=1) value_return=Active_;
                    else{
                        if(in_on_string=="") in_on_string+=to_string(data[i]);
                        else in_on_string+=","+to_string(data[i]);
                    }
                }else value_return=Active_;
            }
        }
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"in_on_set:"<<in_on<<endl;
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"in_on_read:"<<in_on_string<<endl;
        // in_off
        static string in_off_string;
        in_off_string="";
        if(in_off!=""){
            data=process_data2(in_off);
            for(int i=0;i<data.size();i++){
                if(data[i]<input_user_status_1.data.size()){
                    if(input_user_status_1.data[data[i]]!=0) value_return=Active_;
                    else{
                        if(in_off_string=="") in_off_string+=to_string(data[i]);
                        else in_off_string+=","+to_string(data[i]);
                    }
                }else value_return=Active_;
            }
        } 
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"in_off_set:"<<in_off<<endl;
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"in_off_read:"<<in_off_string<<endl;
        // in pullup
        if(in_pullup!="") {
            data=process_data2(in_pullup);
            for(int i=0;i<data.size();i++){
                if(data[i]<input_user_status_1.data.size() & data[i]<input_user_status_2.data.size()){
                    if(input_user_status_1.data[data[i]]==1 & input_user_status_2.data[data[i]]==0 ){            
                        is_have=0;
                        if(in_pullup_string !="") data2=process_data2(in_pullup_string);
                        for(int j=0;j<data2.size();j++){
                            if(data2[j]==data[i]) is_have=1;
                        }
                        if(is_have==0){
                            if(in_pullup_string=="") in_pullup_string+=to_string(data[i]);
                            else in_pullup_string+=","+to_string(data[i]);
                        } 

                    }
                }else value_return=Active_;
            }
            if(in_pullup_string !="") data2=process_data2(in_pullup_string);
            else data2.resize(0);
            if(data2.size()!=data.size()) value_return=Active_; 
        }
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"in_pullup_set:"<<in_pullup<<endl;
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"in_pullup_read:"<<in_pullup_string<<endl;
        // in pulldown
        if(in_pulldown!="") {
            data=process_data2(in_pulldown);
            for(int i=0;i<data.size();i++){
                if(data[i]<input_user_status_1.data.size() & data[i]<input_user_status_2.data.size()){
                    if(input_user_status_1.data[data[i]]==0 & input_user_status_2.data[data[i]]==1 ){            
                        is_have=0;
                        if(in_pulldown_string !="") data2=process_data2(in_pulldown_string);
                        for(int j=0;j<data2.size();j++){
                            if(data2[j]==data[i]) is_have=1;
                        }
                        if(is_have==0){
                            if(in_pulldown_string=="") in_pulldown_string+=to_string(data[i]);
                            else in_pulldown_string+=","+to_string(data[i]);
                        } 

                    }
                }else value_return=Active_;
            }
            if(in_pulldown_string !="") data2=process_data2(in_pulldown_string);
            else data2.resize(0);
            if(data2.size()!=data.size()) value_return=Active_; 
        }
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"in_pulldown_set:"<<in_pulldown<<endl;
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"in_pulldown_read:"<<in_pulldown_string<<endl;
        //
        if(value_return==Finish_){
            in_pullup_string="";
            in_pulldown_string="";
            data2.resize(0);
        }
    }else{
        value_return=action;
        in_pullup_string="";
        in_pulldown_string="";
        data2.resize(0);
    } 
    return value_return;
}
void gpio_::reset(){
    in_pullup_string="";
    in_pulldown_string="";
}