using namespace std;
//
void variable_::process_data(){
    static string_Iv2 data_return;
    data_return.detect(data,"~","=","~");
    for(int i=0;i<data_return.data1.size();i++){
        if(data_return.data1[i]=="command_action")    command_action=data_return.data2[i];
        if(data_return.data1[i]=="name_variable")     name_variable=data_return.data2[i];
        if(data_return.data1[i]=="focus_value")       focus_value=data_return.data2[i];
    }
}
void variable_::print(int n){
    num_tab=n;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"Data:"<<data<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"command_action:"<<command_action<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"name_variable:"<<name_variable<<endl;
    for(int i=0;i<num_tab;i++) cout<<"\t ";
    cout<<"focus_value:"<<focus_value<<endl;
}
int variable_::action(int action){
    static int value_return;
    if(action==Active_){
        static int is_have,is_have2;
        is_have=0;
        is_have2=0;
        for(int i=0;i<my_vars_global.var.size();i++){
            if(my_vars_global.var[i].name==focus_value){
                is_have=i+1;
            }
            if(my_vars_global.var[i].name==name_variable){
                is_have2=i+1;
            }
        }
        //
        static variable local_variable; 
        if(command_action=="new"){
            local_variable.name=name_variable;
            local_variable.data=0;
            my_vars_global.add_var(local_variable);
            value_return=Finish_;
        }
        // ==
        if(command_action=="equal_as"){
            if(is_have2==0){
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Error not have variable"<<endl; 
                value_return=Error_;
            }else{
                if(is_have!=0){
                    if(my_vars_global.var[is_have2-1].data==my_vars_global.var[is_have-1].data){
                        for(int i=0;i<num_tab;i++) cout<<"\t ";
                        cout<<"Equal variable is true"<<endl;
                        value_return=True_;
                    }
                    else{
                        for(int i=0;i<num_tab;i++) cout<<"\t ";
                        cout<<"Equal variable is false"<<endl;
                        value_return=False_;
                    }
                }
                else{
                    if(my_vars_global.var[is_have2-1].data==stof(focus_value)){
                        for(int i=0;i<num_tab;i++) cout<<"\t ";
                        cout<<"Equal variable is true"<<endl;
                        value_return=True_;
                    }
                    else{
                        for(int i=0;i<num_tab;i++) cout<<"\t ";
                        cout<<"Equal variable is false"<<endl;
                        value_return=False_;
                    }
                }
            }
        }
        // !=
        if(command_action=="equal_not"){
            if(is_have2==0){
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Error not have variable"<<endl; 
                value_return=Error_;
            }else{
                if(is_have!=0){
                    if(my_vars_global.var[is_have2-1].data!=my_vars_global.var[is_have-1].data){
                        for(int i=0;i<num_tab;i++) cout<<"\t ";
                        cout<<"Equal  not variable is true"<<endl;
                        value_return=True_;
                    }
                    else{
                        for(int i=0;i<num_tab;i++) cout<<"\t ";
                        cout<<"Equal not  variable is false"<<endl;
                        value_return=False_;
                    }
                }
                else{
                    if(my_vars_global.var[is_have2-1].data!=stof(focus_value)){
                        for(int i=0;i<num_tab;i++) cout<<"\t ";
                        cout<<"Equal not variable is true"<<endl;
                        value_return=True_;
                    }
                    else{
                        for(int i=0;i<num_tab;i++) cout<<"\t ";
                        cout<<"Equal not variable is false"<<endl;
                        value_return=False_;
                    }
                }
            }
        }
        if(command_action=="smaller_as"){

        }
        if(command_action=="bigger_as"){

        }
        if(command_action=="equal"){
            if(is_have2==0){
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Error not have variable"<<endl; 
                value_return=Error_;
            }else{
                if(is_have!=0) 
                my_vars_global.var[is_have2-1].data=my_vars_global.var[is_have-1].data;
                else 
                my_vars_global.var[is_have2-1].data=stof(focus_value);
                value_return=Finish_;
            }
            
        }
        if(command_action=="equal_+"){
            if(is_have2==0){
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Error not have variable"<<endl; 
                value_return=Error_;
            }else{
                if(is_have!=0) 
                my_vars_global.var[is_have2-1].data+=my_vars_global.var[is_have-1].data;
                else 
                my_vars_global.var[is_have2-1].data+=stof(focus_value);
                value_return=Finish_;
            }
        }
        if(command_action=="equal_-"){
            if(is_have2==0){
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Error not have variable"<<endl; 
                value_return=Error_;
            }else{
                if(is_have!=0) my_vars_global.var[is_have2-1].data-=my_vars_global.var[is_have-1].data;
                else my_vars_global.var[is_have2-1].data-=stof(focus_value);
                value_return=Finish_;
            }
        }
    }else{
        value_return=action;
    }
    return value_return;
}
void variable_::reset(){
    
}