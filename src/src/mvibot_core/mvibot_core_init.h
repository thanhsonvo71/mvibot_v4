using namespace std;
#if !defined(mvibot_core_init)
    // mode operating for robot
    string mode;
    string mvibot_seri;
    // param robot 
    float R=0.08,L=0.597;
    float gear=25;
    float v_max=0.5,w_max=0.314;
    float ax=1.0,aw=3.0;
    float robot_shutdown=0;
    float volume=100;
    float low_battery=5;
    // hook param
    float encoder,encoder2;
    float encoder_offset=243;
    float encoder_dir=1;
    float d_hook=1.35;
    int hook_switch=0;
    // motor 
    int motor_enable=1;
    int motor_break=1;
    int motor_reset=0;
    int motor_right_disable,motor_left_disable;
    int motor_right_break,motor_left_break;
    int motor_right_state_live,motor_left_state_live;
    int motor_right_state_error,motor_left_state_error;
    float ivr=0,ivl=0,vr_out=0,vl_out=0;
    float kpvrl=0.015,kivrl=1.5,kdvrl=0;
    float vr,vl;
    float tr,tl;
    float vr_set,vl_set;
    float torqueL_set=1.92,torqueR_set=1.92;
    float vrl_max=12.0; //rad/s
    float ts_pid;
    // sensor robot
    int mvibot_sensor_ready=0;
    float ts_scan_sensor;
    int radar1_live_status=0;
    int radar2_live_status=0;
    int camera1_live_status=0;
    int camera2_live_status=0;
    int battery_live_status=0;
    int battery_small_live_status=0;
    // speed control
    int motor_stop=0;
    int robot_emg;
    float v_set1,v_set2,v_set3,w_set1,w_set2,w_set3;
    float ts_speed_control;
    float time_out_cmd_vel;
    // process data uart 
    std::vector<uint8_t> data_receive={};
    std::vector<uint8_t> data_tran={};
    // battery 
    float battery_soc=-1;
    float battery_vol=0;
    float battery_cycle=0;
    float battery_mah_now=0;
    float battery_mah_max=0;
    float battery_guard=0;
    float battery_temperature=0;
    float battery_num_cell=0;
    float battery_current=0;
    float battery_cell[8];
    int battery1_charge;
    // small battery
    float battery_small_soc=-1;
    float battery_small_vol=0;
    float low_battery_small=5;
    float battery_small_temperature=0;
    float battery_small_current=0;
    float battery_small_mah_now=0;
    float battery_small_mah_max=0;
    float battery_small_num_cell=0;
    int battery2_charge;
    int battery_status_charge;
    //gpio
    std_msgs::Float32MultiArray input_user;
    std_msgs::Float32MultiArray output_user;
    // led & color
    float green,red,blue;
    float led_r,led_l,led_f,led_b;
    // odom
    nav_msgs::Odometry odom_wheel;
    float x_wheel=0,y_wheel=0,theta_wheel=0;
    float vx_wheel,vy_wheel,vth_wheel;
    sensor_msgs::Imu imu_msg;
    float distance_robot=0;
    float distance_wheel_right=0;
    float distance_wheel_left=0;
    // music
    int status_music_n=0;
    int start_music_n=0;
    // update software
    float software_update=0;
    float software_update_status=0;
    string robot_config_string;
    // rqt data
    std_msgs::Float32MultiArray rqt_data;
    #define mvibot_core_init 1
#endif