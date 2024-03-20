using namespace std;
void multiple_step_II::process_data(){
    if(data!=""){
        if(data[0]==start_char){
            int count_start,count_start_f;
            string detect;
            string detect2;
            vector<string> multipe_detect;
            //
            count_start=0;
            count_start_f=0;
            detect="";
            detect2="";
            for(int i=0;i<data.length();i++){
                count_start_f=count_start;
                if(count_start!=0 ) {
                    if(!(data[i]==end_char & count_start==1)) detect+=data[i];
                }
                if(data[i]==start_char) {
                    count_start++;
                }
                if(data[i]==end_char) {
                    count_start--;
                }
                if(count_start==0 & count_start_f !=0){
                    multipe_detect.resize(multipe_detect.size()+1);
                    multipe_detect[multipe_detect.size()-1]=detect;
                    detect="";
                }
                if(count_start_f==0 & count_start==1){
                    if(detect2!=""){
                        multipe_detect.resize(multipe_detect.size()+1);
                        multipe_detect[multipe_detect.size()-1]=detect2;
                        detect2="";
                    }

                }
            }
            //
            if(detect!=""){
                multipe_detect.resize(multipe_detect.size()+1);
                multipe_detect[multipe_detect.size()-1]=detect;
                detect="";
            }
            // //
            my_multiple_step_II.resize(multipe_detect.size());
            for(int i=0;i<multipe_detect.size();i++){
                my_multiple_step_II[i]= new multiple_step_II;
                my_multiple_step_II[i]->data=multipe_detect[i];
                my_multiple_step_II[i]->process_data();
            }
        }else{
           my_step=new step_II;
           my_step->data=data;
           my_step->process_data();
        }
    }
}
void multiple_step_II::print(int n){
    num_tab=n;
    if(my_multiple_step_II.size()!=0){
        for(int i=0;i<my_multiple_step_II.size();i++){
            //
            for(int j=0;j<n;j++) cout<<"\t";
            cout<<"Step->"<<i<<"/"<<"Total step= "<<my_multiple_step_II[i]->num_small_step_action<<endl;
            my_multiple_step_II[i]->print(n+1);
        }
    }else{
        if(my_step!=nullptr){
            my_step->print(n+1);
        }
    }
}
int multiple_step_II::action(int action){
    static int value_return;
    //
    if(action==Active_){
        if(my_multiple_step_II.size()!=0){
            //
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Action Step ->"<<now_action_step<<endl;
            value_return=my_multiple_step_II[now_action_step]->action(Active_);
            //
            if(value_return==Finish_ | value_return==True_ | value_return==False_){
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Finish Step ->"<<now_action_step<<endl;
            }
            else if(value_return==Error_){
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Error Step ->"<<now_action_step<<endl;
            }
        }else {
            if(my_step!=nullptr) value_return=my_step->action(Active_);
            else value_return=Finish_;
        }
    }else{
        if(my_multiple_step_II.size()!=0){
            value_return=my_multiple_step_II[now_action_step]->action(action);
        }else {
            if(my_step!=nullptr)
            value_return=my_step->action(action);
        }
    }
    //
    if((value_return==Finish_ | value_return==True_ | value_return==False_ ) &  my_multiple_step_II.size()!=0 ){
        now_action_step++;
        if(now_action_step>=my_multiple_step_II.size()){
            now_action_step=0;  
        }else value_return=Active_;
    }
    return value_return;
}
void multiple_step_II::reset(){
    now_action_step=0;
    if(my_multiple_step_II.size()!=0){
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"Reset Step"<<endl;
        for(int i=0;i<my_multiple_step_II.size();i++) my_multiple_step_II[i]->reset();
    }else{
        for(int i=0;i<num_tab;i++) cout<<"\t ";
        cout<<"Reset Step"<<endl;
        if(my_step!=nullptr) my_step->reset();
    }
}
int multiple_step_II::set_id(int n){
    static int res;
    res=n;
    if(my_multiple_step_II.size()!=0){
        for(int i=0;i<my_multiple_step_II.size();i++){
            res=my_multiple_step_II[i]->set_id(res);
        }
    }else{
        if(my_step!=nullptr)
        res=my_step->set_id(n);
        else res=n;
    }
    num_small_step_action=res-n;
    return res;
}
void multiple_step_II::delete_free(){
    if(my_multiple_step_II.size()==0 & my_step!=nullptr){
        my_step->delete_free();
        delete my_step;
    }
    for(int i=0;i<my_multiple_step_II.size();i++){
        if(my_multiple_step_II[i]!=nullptr){
            my_multiple_step_II[i]->delete_free();
            delete my_multiple_step_II[i];
        }
    }
    my_multiple_step_II.clear();
}