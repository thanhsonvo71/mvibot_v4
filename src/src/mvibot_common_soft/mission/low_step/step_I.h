using namespace std;
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
void step_I::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"","|","");
    for(int i=0;i<data_return.data1.size();i++){
        static string_Iv2 data_return2;
        data_return2.detect(data_return.data1[i],"",":","");
        if(data_return2.data1.size()==2){
            if(data_return2.data1[0]=="name")       name_step=data_return2.data1[1];
            if(data_return2.data1[0]=="time_out")   time_out_step=stof(data_return2.data1[1]);
            if(data_return2.data1[0]=="mode")       mode_step=data_return2.data1[1];
            if(data_return2.data1[0]=="data"){
                if(mode_step=="sound"){
                    sound_step.data=data_return2.data1[1];
                    sound_step.process_data();
                }
                if(mode_step=="marker") {
                    marker_step.data=data_return2.data1[1];
                    marker_step.process_data();
                }
                if(mode_step=="position"){
                    position_goal_step.data=data_return2.data1[1];
                    position_goal_step.process_data();
                } 
                if(mode_step=="sleep"){
                    sleep_step.data=data_return2.data1[1];
                    sleep_step.process_data();
                }
                if(mode_step=="footprint") {
                    footprint_step.data=data_return2.data1[1];
                    footprint_step.process_data();
                }
                if(mode_step=="gpio"){
                    gpio_step.data=data_return2.data1[1];
                    gpio_step.process_data();
                } 
                if(mode_step=="gpio_module"){
                    gpio_module_step.data=data_return2.data1[1];
                    gpio_module_step.process_data();
                }
                // if(mode_step=="follow_path") {
                //     follow_path_step.data=data_return2.data1[1];
                // }
                if(mode_step=="break"){
                    break_step.data=data_return2.data1[1];
                    break_step.process_data();
                }   
                if(mode_step=="variable"){
                    variable_step.data=data_return2.data1[1];
                    variable_step.process_data();
                }
            }
        }
    }
}
void step_I::print(int n){
    num_tab=n;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"step-------------------------------------------------"<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"name:"<<name_step<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"id: "<<id<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"mode:"<<mode_step<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"time_out:"<<time_out_step<<endl;   
    //
    //
    if(mode_step=="sound") sound_step.print(n+1);
    if(mode_step=="marker") marker_step.print(n+1);
    if(mode_step=="position") position_goal_step.print(n+1);
    if(mode_step=="sleep") sleep_step.print(n+1);
    if(mode_step=="footprint") footprint_step.print(n+1);
    if(mode_step=="gpio") gpio_step.print(n+1);
    if(mode_step=="gpio_module") gpio_module_step.print(n+1);
    // if(mode_step=="follow_path") follow_path_step.print();
    if(mode_step=="break") break_step.print(n+1);
    if(mode_step=="variable") variable_step.print(n+1);
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"------------------------------------------------------"<<endl;
    cout<<endl;
}
int step_I::action(int action){
    static int value_return;
    if(action==Cancel_) time_action_step=0;
    if(time_action_step > time_out_step & time_out_step!=-1){
        time_action_step=time_out_step;
        action=Cancel_;
        if(mode_step=="sound")          value_return=sound_step.action(action);
        if(mode_step=="marker")         value_return=marker_step.action(action);
        if(mode_step=="position")       value_return=position_goal_step.action(action);
        if(mode_step=="sleep")             value_return=sleep_step.action(action);
        if(mode_step=="footprint")      value_return=footprint_step.action(action);
        if(mode_step=="gpio")           value_return=gpio_step.action(action);
        if(mode_step=="gpio_module")    value_return=gpio_module_step.action(action);
        // if(mode_step=="follow_pạth")    value_return=follow_path_step.action(action);
        if(mode_step=="break")          value_return=break_step.action(action);
        if(mode_step=="variable")       value_return=variable_step.action(action);
        //
        value_return=Error_;
    }else{
        //if(time_out_step==-1 | action!=Active_) time_action_step=0;
        if(action!=Cancel_){
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Step name:"<<name_step<<endl;
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"id: "<<id<<endl;
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"mode:"<<mode_step<<endl;
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"time_out:"<<time_out_step<<endl;
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"time_action:"<<time_action_step<<endl;
        }
        //
        if(mode_step=="sound")          value_return=sound_step.action(action);
        if(mode_step=="marker")         value_return=marker_step.action(action);
        if(mode_step=="position")       value_return=position_goal_step.action(action);
        if(mode_step=="sleep")          value_return=sleep_step.action(action);
        if(mode_step=="footprint")      value_return=footprint_step.action(action);
        if(mode_step=="gpio")           value_return=gpio_step.action(action);
        if(mode_step=="gpio_module")    value_return=gpio_module_step.action(action);
        // if(mode_step=="follow_pạth")    value_return=follow_path_step.action(action);
        if(mode_step=="break")          value_return=break_step.action(action);
        if(mode_step=="variable")       value_return=variable_step.action(action);
        //
        if(value_return==Finish_ | value_return==Error_ | value_return==True_ | value_return== False_) time_action_step=0;
    }
    //
    if(name_step!="wake_up" & name_step!="stop" & name_step!="skip") type_action_step=mode_step;
    if(name_step!="wake_up" & name_step!="stop" & name_step!="skip") {
        step_action_information=to_string(id+1);
        infor_action_step=data;
    }
    if(action==Active_ & value_return!=Finish_) time_action_step+=(float)ts_mission_step_scan;
    //
    return value_return;
}
void step_I::reset(){
    time_action_step=0;
    //
    if(mode_step=="sleep")          sleep_step.reset();
    if(mode_step=="break")          break_step.reset();
    if(mode_step=="variable")       variable_step.reset();
    if(mode_step=="sound")          sound_step.reset();
    if(mode_step=="foootprint")     footprint_step.reset();
    if(mode_step=="gpio")           gpio_step.reset();
    if(mode_step=="gpio_module")    gpio_module_step.reset();
    if(mode_step=="marker")         marker_step.reset();
    if(mode_step=="position")       position_goal_step.reset();
}
int step_I::set_id(int n){
    id=n;
    return n+1;
}