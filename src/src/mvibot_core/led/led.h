#include"../../common/libary/libary_basic.h"
using namespace std;
// led & color
extern float green,red,blue;
extern float led_r,led_l,led_f,led_b;
// mode
extern string mode;
extern int  motor_left_state_error,motor_left_state_live,motor_right_state_error,motor_right_state_live;
extern float software_update_status,software_update;
extern float battery_soc;
extern float low_battery;
extern int mvibot_sensor_ready;
extern int battery_status_charge;
extern int n_re_connect;
void set_color_led(float cred, float cgreen, float cblue){
    red=cred;
    blue=cblue;
    green=cgreen;
}
void led_control(){
    if(mode=="slam"){
        set_color_led(100,0,25); //100 0 25
        led_r=3;
        led_l=3;
        led_b=3;
    }
    if(n_re_connect>=10){
        set_color_led(100,20,0);
        led_r=2;
        led_l=2;
        led_b=2;
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
     if(battery_status_charge==1){
    	set_color_led(0,0,100);
        led_r=1;
        led_l=1;
        led_b=1;
    }
    if(software_update==1){
        set_color_led(0,0,100);
        led_r=2;
        led_l=2;
        led_b=2;
    }   
}
