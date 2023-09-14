#include"../../common/libary/libary_basic.h"
using namespace std;
//
extern int motor_enable;
extern int motor_right_disable,motor_left_disable;
extern int motor_right_state_live,motor_left_state_live;
extern int motor_right_state_error,motor_left_state_error;
extern float ivr,ivl,vr_out,vl_out;
extern float kpvrl,kivrl,kdvrl;
extern float vr,vl;
extern float vr_set,vl_set;
extern float torqueL_set,torqueR_set;
extern int motor_stop;
extern float v_set1,v_set2,v_set3,w_set1,w_set2,w_set3;
extern float ts_speed_control;
extern float time_out_cmd_vel;
//
void speed_robot_control(){
    // time out cmd_vel
    if(v_set1 !=0 | w_set1 !=0){
        if(time_out_cmd_vel>=0.5){
            v_set1=0;
            w_set1=0;
            time_out_cmd_vel=0.5;
        }else time_out_cmd_vel+=(float)ts_speed_control;
    }else time_out_cmd_vel=0;
    // motor stop
    static int stop;
    stop=0;
    //
    if(motor_enable==0) stop=1;
    if(motor_stop==1)   stop=1;
    if(motor_left_state_live==0 | motor_right_state_live==0) stop=1;
    if(motor_right_disable==0 | motor_left_disable==0) stop=1;
    if(motor_left_state_error!=0 | motor_right_state_error!=0) stop=1;
    if(stop==1){
        // reset linear 
        v_set1=0;
        v_set2=0;
        v_set3=0;
        // reset rotary
        w_set1=0;
        w_set2=0;
        w_set3=0;
        // reset pid value
        vr_set=0;
        vl_set=0;
        vl_out=0;
		vr_out=0; 
		ivl=0;
		ivr=0;
        if(motor_stop==1) motor_stop=0;
    }else{
        if(mvibot_sensor_ready==0){
            v_set1=0;
            w_set1=0;
        }
        // acceleration for robot
        if(v_set1>v_set2) {
            v_set2+=ax*(float)ts_speed_control;
            if(v_set2>v_set1) v_set2=v_set1;
        }
        if(v_set1<v_set2){
            v_set2-=ax*(float)ts_speed_control;
            if(v_set2<v_set1) v_set2=v_set1;
        }
        if(w_set1>w_set2) {
                w_set2+=aw*(float)ts_speed_control;
                if(w_set2>w_set1) w_set2=w_set1;
        }
        if(w_set1<w_set2){
                w_set2-=aw*(float)ts_speed_control;
                if(w_set2<w_set1) w_set2=w_set1;
        }
        // if have iso safe, must write here -> v_set3 w_set3
        v_set3=v_set2;
        w_set3=w_set2;
    }
}