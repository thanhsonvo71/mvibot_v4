using namespace std;
//
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
void sound_::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"~","=","~");
    for(int i=0;i<data_return.data1.size();i++){
        if(data_return.data1[i]=="music_name")     music_name=data_return.data2[i];
        if(data_return.data1[i]=="music_mode")     music_mode=stof_f(data_return.data2[i]);
        if(data_return.data1[i]=="music_start")    music_start=stof_f(data_return.data2[i]); 
    }
}
//
void sound_::print(int n){
    //
    num_tab=n;    
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"Data:"<<data<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"music_mode:"<<music_mode<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"music_start:"<<music_start<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"music_name:"<<music_name<<endl;
}
//
int sound_::action(int action){
    if(action==Active_){
        for(int j=0;j<num_tab;j++) cout<<"\t";
        cout<<"Action sound"<<endl;
        //
        if(action_mission==Error_){
            if(music_start==0) sound2=0;
            else{
                sound2=music_mode;
            }
        }else{
            if(music_start==0) sound=0;
            else{
                sound=music_mode;
            }
        }
        return Finish_;
    } else  return action;
}
void sound_::reset(){

}