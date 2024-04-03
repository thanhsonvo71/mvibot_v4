using namespace std;
//
class telegram_{
	public:
        string data;
        //
        string token;
        string chat_id;
        string msg;
        //
        int status=0;
        int num_tab;
        //
        void print(int n);
        void process_data();
        int action(int action); 
        void reset();
};
void telegram_::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"~","=","~");
    for(int i=0;i<data_return.data1.size();i++){
        if(data_return.data1[i]=="token")     token=data_return.data2[i];
        if(data_return.data1[i]=="chat_id")   chat_id=data_return.data2[i];
        if(data_return.data1[i]=="msg")       msg=data_return.data2[i];
    }
}
//
void telegram_::print(int n){
    num_tab=n;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"Data:"<<data<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"token:"<<token<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"chat_id:"<<chat_id<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"msg:"<<msg<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"Status:"<<status<<endl;
}
//
int telegram_::action(int action){
    static int value_return;
    if(action==Active_){
        string http="";
        http="";
        http=http+"curl \"https:/"+"/api.telegram.org/bot"+token+"/sendMessage?chat_id="+chat_id+"&text="+msg+"\"";
        exec(http);
        status=Finish_;
    }else{
        status=action;
    }
    value_return=status;
    return value_return;
}
void telegram_::reset(){
   
}
//