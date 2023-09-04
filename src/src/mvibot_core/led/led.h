#include"../../common/libary/libary_basic.h"
#include"../mvibot_core_init.h"
using namespace std;
//
void set_color_led(float cred, float cgreen, float cblue){
    red=cred;
    blue=cblue;
    green=cgreen;
}
void led_control(){
    if(mode=="slam"){
        set_color_led(100,0,25);
        led_r=3;
        led_l=3;
        led_b=3;
    }
    if((motor_left_state_error !=0 & motor_left_state_live==1) | (motor_right_state_error !=0 & motor_right_state_live==1) | software_update_status==1){
        set_color_led(100,0,0);
        led_r=1;
        led_l=1;
        led_b=1;
    }
    if(battery_soc <= low_battery & battery_soc!=-1){
        set_color_led(100,0,0);
        led_r=2;
        led_l=2;
        led_b=2;
    }else{
        if(mvibot_sensor_ready==0) {
            set_color_led(100,100,100);
            led_r=1;
            led_l=1;
            led_b=1;
        }
    }
    if(software_update==1){
        set_color_led(0,0,100);
        led_r=2;
        led_l=2;
        led_b=2;
    }
   
}