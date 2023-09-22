using namespace std;
//
class variable_local{
    public:
        string name;
        float data;
};
class multiple_variable_local{
    public:
        vector<variable_local> var;
        //
        void add_var(variable_local new_var){
            static int is_have;
            is_have=0;
            for(int i=0;i<var.size();i++){
                 if(var[i].name==new_var.name){
                    is_have=1;
                    var[i].data=new_var.data;
               }
            }
            if(is_have==0){
                var.resize(var.size()+1);
                var[var.size()-1]=new_var;
            }
        }
        //
        void update_var(variable_local var_update){
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
//
multiple_variable_local   my_vars_local;
//
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
        for(int i=0;i<my_vars_local.var.size();i++){
            if(my_vars_local.var[i].name==focus_value){
                is_have=i+1;
            }
            if(my_vars_local.var[i].name==name_variable){
                is_have2=i+1;
            }
        }
        // new variable
        if(command_action=="new"){
            if(is_have2==0){
                variable_local local_variable; 
                local_variable.name=name_variable;
                if(is_have==0) local_variable.data=stof_f(focus_value);
                else local_variable.data=my_vars_local.var[is_have-1].data;
                my_vars_local.add_var(local_variable);
            }
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
                    if(my_vars_local.var[is_have2-1].data==my_vars_local.var[is_have-1].data){
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
                    if(my_vars_local.var[is_have2-1].data==stof(focus_value)){
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
                    if(my_vars_local.var[is_have2-1].data!=my_vars_local.var[is_have-1].data){
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
                    if(my_vars_local.var[is_have2-1].data!=stof(focus_value)){
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
        // =
        if(command_action=="equal"){
            if(is_have2==0){
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Error not have variable"<<endl; 
                value_return=Error_;
            }else{
                if(is_have!=0) 
                my_vars_local.var[is_have2-1].data=my_vars_local.var[is_have-1].data;
                else 
                my_vars_local.var[is_have2-1].data=stof(focus_value);
                value_return=Finish_;
            }
            
        }
        // +=
        if(command_action=="equal_+"){
            if(is_have2==0){
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Error not have variable"<<endl; 
                value_return=Error_;
            }else{
                if(is_have!=0) 
                my_vars_local.var[is_have2-1].data+=my_vars_local.var[is_have-1].data;
                else 
                my_vars_local.var[is_have2-1].data+=stof(focus_value);
                value_return=Finish_;
            }
        }
        // -=
        if(command_action=="equal_-"){
            if(is_have2==0){
                for(int i=0;i<num_tab;i++) cout<<"\t ";
                cout<<"Error not have variable"<<endl; 
                value_return=Error_;
            }else{
                if(is_have!=0) my_vars_local.var[is_have2-1].data-=my_vars_local.var[is_have-1].data;
                else my_vars_local.var[is_have2-1].data-=stof(focus_value);
                value_return=Finish_;
            }
        }
        // Reset
        if(command_action=="reset"){
           my_vars_local.reset_all();
           value_return=Finish_;
        }
        // delete 
        if(command_action=="delete"){
           my_vars_local.delete_all();
           value_return=Finish_;
        }
    }else{
        value_return=action;
    }
    return value_return;
}
void variable_::reset(){
    
}