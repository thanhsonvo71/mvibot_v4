using namespace std;
//
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
void break_::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"~","=","~");
}
//
void break_::print(int n){
    num_tab=n;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"Break!"<<endl;
}
//
int break_::action(int action){
    static int value_return;
    if(action==Active_){
        value_return=Break_;
        for(int i=0;i<num_tab;i++) cout<<" \t";
        cout<<"Break!"<<endl;
    }else{
        value_return=action;
    }
    return value_return;
}
void break_::reset(){
    
}
//