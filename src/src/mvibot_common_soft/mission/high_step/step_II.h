using namespace std;
void step_II::process_data(){
    static string detect;
    static string detect2;
    detect="";
    detect2="";
    static int check;
    check=0;
    //
    for(int i=0;i<data.length();i++){
        if(data[i]!='#') {
            detect+=data[i];
        }
        else{
            check=1;
            for(int j=i+1;j<data.length();j++){
                detect2+=data[j];
                i=j;
            }
        }
    }
    // step action 
    if(check==1) mode=detect;
    else {
        mode="smallest_step";
        my_smallest_step=new multiple_step_I;
        my_smallest_step->data=detect;
        my_smallest_step->process_data();
    }
    //
    if(detect=="if_else"){
        my_if_else_step=new if_else_step;
        my_if_else_step->data=detect2;
        my_if_else_step->process_data();
    }
    if(detect=="try_catch"){
        my_try_catch_step=new try_catch_step;
        my_try_catch_step->data=detect2;
        my_try_catch_step->process_data();
    }
    if(detect=="normal"){
        my_normal_step=new normal_step;
        my_normal_step->data=detect2;
        my_normal_step->process_data();
    }
    //
    if(detect=="logic_and"){
        my_logic_and_step=new logic_and_step;
        my_logic_and_step->data=detect2;
        my_logic_and_step->process_data();
    }
    if(detect=="logic_or"){
        my_logic_or_step=new logic_or_step;
        my_logic_or_step->data=detect2;
        my_logic_or_step->process_data();
    }
    if(detect=="while"){
        my_while_step=new while_step;
        my_while_step->data=detect2;
        my_while_step->process_data();
    }
}
void step_II::print(int n){
    num_tab=n;
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"Mode "<<mode<<endl;
    //
    for(int j=0;j<n;j++) cout<<"\t";
    cout<<"Data "<<data<<endl;
    //
    if(mode=="if_else")         my_if_else_step->print(n+1);
    if(mode=="normal")          my_normal_step->print(n+1);
    if(mode=="try_catch")       my_try_catch_step->print(n+1);
    if(mode=="smallest_step")   my_smallest_step->print(n+1);
    if(mode=="while")           my_while_step->print(n+1); 
    //
    if(mode=="logic_and")   my_logic_and_step->print(n+1);
    if(mode=="logic_or")    my_logic_or_step->print(n+1);

}
int step_II::action(int n){
    static int value_return;
    //
    if(mode=="smallest_step"){
        if(n==Active_){
            for(int i=0;i<num_tab;i++) cout<<"\t ";
            cout<<"Smallest_step"<<endl;
        }
        value_return=my_smallest_step->action(n);
    }
    if(mode=="if_else"){
        if(n==Active_){
             for(int i=0;i<num_tab;i++) cout<<"\t ";
             cout<<"if_else"<<endl;
        }  
        value_return=my_if_else_step->action(n);
    }
    if(mode=="try_catch"){
        if(n==Active_){
             for(int i=0;i<num_tab;i++) cout<<"\t ";
             cout<<"try_catch"<<endl;
        }
        value_return=my_try_catch_step->action(n);
    }
    if(mode=="normal"){
        if(n==Active_){
             for(int i=0;i<num_tab;i++) cout<<"\t ";
             cout<<"normal"<<endl;
        }
        value_return=my_normal_step->action(n);
    }
    if(mode=="while"){
        if(n==Active_){
             for(int i=0;i<num_tab;i++) cout<<"\t ";
             cout<<"while"<<endl;
        }
        value_return=my_while_step->action(n);
    }
    //
    if(mode=="logic_and"){
        if(n==Active_){
             for(int i=0;i<num_tab;i++) cout<<"\t ";
             cout<<"logic_and"<<endl;
        }
        value_return=my_logic_and_step->action(n);
    }
    if(mode=="logic_or"){
        if(n==Active_){
             for(int i=0;i<num_tab;i++) cout<<"\t ";
             cout<<"logic_or"<<endl;
        }
        value_return=my_logic_or_step->action(n);
    }
    return value_return;
}
void step_II::reset(){
    //
    if(mode=="smallest_step")   my_smallest_step->reset();
    if(mode=="if_else")         my_if_else_step->reset();
    if(mode=="try_catch")       my_try_catch_step->reset();
    if(mode=="normal")          my_normal_step->reset();
    if(mode=="while")           my_while_step->reset();
    //
    if(mode=="logic_and")       my_logic_and_step->reset();
    if(mode=="logic_or")        my_logic_or_step->reset();
}
int step_II::set_id(int n){
    static int value_return;
    //
    if(mode=="smallest_step")   value_return=my_smallest_step->set_id(n);
    if(mode=="if_else")         value_return=my_if_else_step->set_id(n);
    if(mode=="try_catch")       value_return=my_try_catch_step->set_id(n);
    if(mode=="normal")          value_return=my_normal_step->set_id(n);
    if(mode=="while")           value_return=my_while_step->set_id(n);
    //
    if(mode=="logic_and")       value_return=my_logic_and_step->set_id(n);    
    if(mode=="logic_or")        value_return=my_logic_or_step->set_id(n);    
    return value_return;
}
void step_II::delete_free(){
    if(mode=="smallest_step"){
        delete my_smallest_step;
    }
    if(mode=="if_else"){
        if(my_if_else_step->condition_step !=nullptr){
            my_if_else_step->condition_step->delete_free();
            delete my_if_else_step->condition_step;
            my_if_else_step->condition_step=nullptr;
        }
        if(my_if_else_step->if_step !=nullptr){
            my_if_else_step->if_step ->delete_free();
            delete my_if_else_step->if_step;
            my_if_else_step->if_step=nullptr;
        }
        if(my_if_else_step->else_step !=nullptr){
            my_if_else_step->else_step ->delete_free();
            delete my_if_else_step->else_step;
            my_if_else_step->else_step=nullptr;
        }
        delete my_if_else_step;
        my_if_else_step=nullptr;
    }
    if(mode=="try_catch"){
        if(my_try_catch_step->try_step !=nullptr){
            my_try_catch_step->try_step->delete_free();
            delete my_try_catch_step->try_step;
            my_try_catch_step->try_step=nullptr;
        }
        if(my_try_catch_step->catch_step !=nullptr){
            my_try_catch_step->catch_step ->delete_free();
            delete my_try_catch_step->catch_step;
            my_try_catch_step->catch_step=nullptr;
        }
        delete my_try_catch_step;
        my_try_catch_step=nullptr;
    }
    if(mode=="normal"){
        if(my_normal_step->normal_step!=nullptr){
            my_normal_step->normal_step->delete_free();
            delete my_normal_step->normal_step;
            my_normal_step->normal_step=nullptr;
        }
        delete my_normal_step;
        my_normal_step=nullptr;
    }
    if(mode=="while"){
        if(my_while_step->condition_step !=nullptr){
            my_while_step->condition_step->delete_free();
            delete my_while_step->condition_step;
            my_while_step->condition_step=nullptr;
        }
        if(my_while_step->do_step !=nullptr){
            my_while_step->do_step->delete_free();
            delete my_while_step->do_step;
            my_while_step->do_step=nullptr;
        }
        delete my_while_step;
        my_while_step=nullptr;
    }
    //
    if(mode=="logic_and"){
        if(my_logic_and_step->logic_A !=nullptr){
            my_logic_and_step->logic_A->delete_free();
            delete my_logic_and_step->logic_A;
            my_logic_and_step->logic_A=nullptr;
        }
        if(my_logic_and_step->logic_B !=nullptr){
            my_logic_and_step->logic_B->delete_free();
            delete my_logic_and_step->logic_B;
            my_logic_and_step->logic_B=nullptr;
        }
        delete my_logic_and_step;
        my_logic_and_step=nullptr;
    }
    if(mode=="logic_or"){
        if(my_logic_or_step->logic_A !=nullptr){
            my_logic_or_step->logic_A->delete_free();
            delete my_logic_or_step->logic_A;
            my_logic_or_step->logic_A=nullptr;
        }
        if(my_logic_or_step->logic_B !=nullptr){
            my_logic_or_step->logic_B->delete_free();
            delete my_logic_or_step->logic_B;
            my_logic_or_step->logic_B=nullptr;
        }
        delete my_logic_or_step;
        my_logic_or_step=nullptr;
    }
}