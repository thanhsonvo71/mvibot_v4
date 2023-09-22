using namespace std;
//
class marker_{
    public:
        string data;
        int num_tab;
		string mode_marker;
		float x_offset;
		float y_offset;
		float yaw_offset;
        int status=0;
        void print(int n);
        void process_data();
        int action(int action);
        void reset();
};
void pub_action_makrer(std_msgs::String data){
    static ros::NodeHandle n;
    static ros::Publisher  pub = n.advertise<std_msgs::String>("/"+mvibot_seri+"/start_marker", 1);
    static float creat_fun=0;
    if(creat_fun==1)
    {
        pub.publish(data);
    } else creat_fun=1;
}
//
void marker_::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"~","=","~");
    for(int i=0;i<data_return.data1.size();i++){
        if(data_return.data1[i]=="mode_marker")    mode_marker=data_return.data2[i];
        if(data_return.data1[i]=="x_offset")       x_offset=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="y_offset")       y_offset=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="yaw_offset")     yaw_offset=stof_f(data_return.data2[i]);
    }
}
void marker_::print(int n){
    num_tab=n;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"Data:"<<data<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"mode_marker:"<<mode_marker<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"x_offset:"<<x_offset<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"y_offset:"<<y_offset<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"yaw_offset:"<<yaw_offset<<endl;
}
int marker_::action(int action){
    //
    static int value_return;
    if(action==Active_){
        if(status==0){
            if(status_action_marker!=0){
                static std_msgs::String stop;
                stop.data="0";
                pub_action_makrer(stop);
            }else status=1;
            value_return=Active_;
        }
        else if(status==1){
            if(status_action_marker==0){
                static std_msgs::String start;
                start.data=data;
                pub_action_makrer(start);
            }else status=2;
            value_return=Active_;
        }
        else if(status==2){
            if(status_action_marker==2){
                status=3;
            }else if(status_action_marker==0){
                static std_msgs::String start;
                start.data="1";
                pub_action_makrer(start);
            }
            value_return=Active_;
        }
        else if(status==3){
            if(status_action_marker!=0){
                static std_msgs::String stop;
                stop.data="0";
                pub_action_makrer(stop);
                value_return=Active_;
            }else{
                status=0;
                value_return=Finish_;
            }
        }
    }else{
        //status=0;
        static std_msgs::String stop;
        stop.data="0";
        if(status_action_marker!=0) pub_action_makrer(stop);
        value_return=action;
    }
    return value_return;
}
void marker_::reset(){
    status=0;
}