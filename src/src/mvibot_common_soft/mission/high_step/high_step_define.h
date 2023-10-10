using namespace std;
//
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
        int status_A=0;
        int status_B=0;
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
        int status_A=0;
        int status_B=0;
        //
        multiple_step_II * logic_A=nullptr;
        multiple_step_II * logic_B=nullptr;
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
        string get_infor(int mode_get);
};
