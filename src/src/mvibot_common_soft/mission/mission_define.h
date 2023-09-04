//the following are UBUNTU/LINUX, and MacOS ONLY terminal color codes.
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


#define Error_   3
#define Active_  1
#define Cancel_  4
#define Finish_  2
#define N_A_     0  
#define Skip_    5
#define True_    6
#define False_   7
#define Wake_up_ 8
#define Stop_    9
#define Break_   10
#define is_robot 1

#define Mission_normal_             0
#define Mission_charge_battery_     1
//
using namespace std;
float ts_mission_step_scan=0.05;
// global var for robot
int action_mission=0;
int sound=0;
int status_action_marker=0;
// class for variable mision
class variable{
    public:
        string name;
        float data;
};
class multiple_variable{
    public:
        vector<variable> var;
        //
        void add_var(variable new_var){
            static int is_have;
            is_have=0;
            for(int i=0;i<var.size();i++){
                 if(var[i].name==new_var.name){
                    is_have=1;
                    var[i].data=0;
               }
            }
            if(is_have==0){
                var.resize(var.size()+1);
                var[var.size()-1]=new_var;
            }
        }
        //
        void update_var(variable var_update){
            static int is_have;
            is_have=0;
            for(int i=0;i<var.size();i++){
               if(var[i].name==var_update.name){
                    is_have=1;
                    var[i].data=var_update.data;
               }
            }
            if(is_have==0){
                var.resize(var.size()+1);
                var[var.size()-1]=var_update;
                var[var.size()-1].data=0;
            }
        }
        //
        void reset_all(){
            for(int i=0;i<var.size();i++){
                var[i].data=0;
            }
        }
        //
        void delete_all(){
            var.resize(0);
        }
        //
        void print(){
            cout<<BOLDCYAN;
            cout<<"Table Global Var~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
            for(int i=0;i<var.size();i++){
               cout<<"var_name:"<<var[i].name<<"="<<var[i].data<<"\t\t\t\t\t\t|"<<endl;
            }
            cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
            cout<<RESET;
        }
        void reset(){
            var.resize(0);
        }
};
// function support
vector<string> detect_data_v1(string data,char start_char,char end_char){
    //
    static vector<string> string_return;
    static string detect,detect2;
    string_return.resize(0);
    //
    detect="";
    detect2="";
    for(int i=0;i<data.length();i++){
        if(data[i]==start_char){
            static int count,count_f;
            count=0;
            count_f=0;
            for(int j=i;j<data.length();j++){
                detect2+=data[j];
                count_f=count;
                if(data[j]==start_char) count++;
                if(data[j]==end_char) count--;
                //
                if(count==0 & count_f!=0){
                    i=j;
                    string_return.resize(string_return.size()+2);
                    string_return[string_return.size()-2]=detect;
                    string_return[string_return.size()-1]=detect2;
                    //
                    detect="";
                    detect2="";
                    break;
                }
            }
        }else  detect+=data[i];
    }
    return string_return;
}
vector<int> process_data2(string data){
    static vector<int> value_return;
    value_return.resize(0);
    //
    static string_Iv2 data_return;
    data_return.detect(data,"",",","");
    for(int i=0;i<data_return.data1.size();i++){
        static int data_int;
        data_int=stoi_f(data_return.data1[i]);
        if(data_int!=-1){
            value_return.resize(value_return.size()+1);
            value_return[value_return.size()-1]=data_int;
        }
    }
    if(data_return.data1.size()==0 & data!=""){
        static int data_int;
        data_int=stoi_f(data);
        if(data_int!=-1){
            value_return.resize(1);
            value_return[0]=data_int;
        }
    }
    return value_return;
}
// creat low class
class sleep_;
class sound_;
class position_;
class marker_;
class gpio_;
class gpio_module_;
class footprint_;
class variable_;
class multiple_variable;
class break_;
class follow_path_;
class step_I;
class multiple_step_I;
//creat high layer
class if_else_step;
class try_catch_step;
class normal_step;
class while_step;
class logic_and_step;
class logic_or_step;
class step_II;
class multiple_step_II;
// creat mission
class mission;
// modify step low layer
class sleep_{
	public:
        string data;
		long double time;
        long double time_sleep;
        int status=0;
        int num_tab;
        //
        void print(int n);
        void process_data();
        int action(int action); 
        void reset();
};
class variable_{
    public:
        string data;
        float  status;
        //
        int num_tab;
        string  command_action;
        string  name_variable;
        string  focus_value;
        void print(int n);
        void process_data();
        int  action(int action); 
        void reset();
};
class break_{
    public:
        string  data;
        int status=0;
        //
        int  num_tab;
        void print(int n);
        void process_data();
        int action(int action); 
        void reset();
};
class sound_{
	public:
        string data;
        string music_name;
        int  num_tab;
	   	float music_start=0;
		float music_mode=0;
        int status=0;
        void print(int n);
        void process_data();
        int action(int action);
        void reset();
};
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
class gpio_module_{
	public:
        string data;
        int num_tab;
        string in_pullup_string="";
        string in_pulldown_string="";
		string gpio_module_name_seri="";
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
class position_{
	public:
        string data;
        string mode;
        int num_tab;
		float x;
		float y;
		float z;
		float w;
        int status=0;
        void print(int n);
        void process_data();
        int action(int action);
        void reset(); 
};
//
class step_I{
	public:
		//
        string          data;
		string 			name_step;
		string 			mode_step;
        int num_tab;
        int id;
		//
		long double		time_out_step=-1;	
		long double	    time_action_step=0;
		//
		sound_			sound_step;
		position_		position_goal_step; 
		marker_			marker_step;
		gpio_			gpio_step;
		gpio_module_	gpio_module_step;
		sleep_ 			sleep_step;
		footprint_		footprint_step;
        // follow_path_    follow_path_step;
        break_          break_step;
        variable_       variable_step;
        //
        void process_data();
        void print(int n);
        int  action(int action); 
        void reset();
        int  set_id(int n);
};
class multiple_step_I{
    public:
        string data;
        int num_tab;
        long n_step=0;
        long action_step=0;
        float time_out;
        float time;
        vector<step_I> 	my_step; 
        void process_data();
        void print(int n);
        int action(int action);
        void reset();
        int set_id(int n);
};
// modify step high layer
class if_else_step{
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
        multiple_step_II * if_step=nullptr;
        multiple_step_II * else_step=nullptr;
};
class try_catch_step{
    public:
        string data;
        //
        void process_data();
        void print(int n);
        int action(int action);
        void reset();
        int set_id(int n);
        int status=0;
        int num_tab;
        //
        multiple_step_II * try_step=nullptr;
        multiple_step_II * catch_step=nullptr;
};
class normal_step{
    public:
        string data;
        int num_tab;
        int status=0;
        void process_data();
        void print(int n);
        int action(int action);
        void reset();
        int set_id(int n);
        //
        multiple_step_II * normal_step=nullptr;
};
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
class logic_and_step{
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
        int status_A;
        int status_B;
        //
        multiple_step_II * logic_A=nullptr;
        multiple_step_II * logic_B=nullptr;
};
class logic_or_step{
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
        int status_A;
        int status_B;
        //
        multiple_step_II * logic_A=nullptr;
        multiple_step_II * logic_B=nullptr;
};
//
class step_II{
    public:
        string data;
        string mode;
        //
        int num_tab;
        void process_data();
        void print(int n);
        int action(int action);
        void reset();
        void delete_free();
        int set_id(int n);
        // action step
        if_else_step*        my_if_else_step=nullptr;
        try_catch_step*      my_try_catch_step=nullptr;
        normal_step*         my_normal_step=nullptr;
        multiple_step_I*     my_smallest_step=nullptr;
        while_step*          my_while_step=nullptr;
        // logic step
        logic_and_step*      my_logic_and_step=nullptr;
        logic_or_step*       my_logic_or_step=nullptr;

};
class multiple_step_II{
    public:
        string data;
        char start_char='{';
        char end_char='}';
        void process_data();
        void print(int n);
        int  action(int action);
        int  set_id(int n);
        void reset();
        void delete_free();
        //
        int num_tab;
        int now_action_step=0;
        int num_step=0;
        int num_small_step_action=0;
        //
        vector<multiple_step_II *>  my_multiple_step_II;
        step_II*                    my_step=nullptr;
};
// modify mission
class mission{
    public:
        string data;
        string name_mission;
        //
        int id_mission=0;
        multiple_step_II*    my_multiple_step_II=nullptr;
        multiple_step_I*     my_configuration=nullptr; 
        //
        void process_data();
        void print(int n);
        int action(int action);
        void reset();
        void delete_free();
};
// multiple_mission
class multiple_mission{
    public:
        string data="";
        vector<mission*> multiple_mission;
        void process_data();
        int action(int action);
        int num_mission_action=-1;
        void print(int n);
        void reset();
        void delete_free();
};
// string name_seri
// string mvibot_seri;
// gpio suppport for mission
string gpio_msg;
std_msgs::Float32MultiArray  input_user_status;
std_msgs::Float32MultiArray  input_user_status_1;
std_msgs::Float32MultiArray  input_user_status_2;
std_msgs::Float32MultiArray  output_user_status;
// infor mission
string type_action_step;
string step_action_information;
string infor_action_step;
int id_action_information;
// var global
multiple_variable   my_vars_global;
// moudle status
class moudle_ {
    public:
        string name;
        std_msgs::Float32MultiArray input_user;
        std_msgs::Float32MultiArray input_user1;
        std_msgs::Float32MultiArray input_user2;
        std_msgs::Float32MultiArray output_user;
};
vector<moudle_>     my_module;