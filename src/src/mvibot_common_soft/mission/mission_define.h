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
#define Continue_ 11

#define Mission_normal_             0
#define Mission_charge_battery_     1
//
using namespace std;
float ts_mission_step_scan=0.05;
// global var for robot
int action_mission=0;
int action_mode_mission;
int sound=0; int sound2=0;
int status_action_marker=0;
// class for variable mision
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
class telegram_;
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
class config_;
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
//
// gpio suppport for mission
string gpio_msg;
std_msgs::Float32MultiArray  input_user_status;
std_msgs::Float32MultiArray  input_user_status_1;
std_msgs::Float32MultiArray  input_user_status_2;
std_msgs::Float32MultiArray  output_user_status;
//
// infor mission
string type_action_step;
string step_action_information;
string infor_action_step;
//int id_action_information;
string step_action_information2;
string infor_action_step2;
//int id_action_information2;
string step_action_information3;
string infor_action_step3;
//int id_action_information3;
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
// data footprint
float footprint_x1=-0.4,footprint_x2=0.4,footprint_y1=-0.345,footprint_y2=0.345;
nav_msgs::Path my_path;