#include"../../common/libary/libary_basic.h"
#include"../mvibot_core_init.h"
#include "../../common/data_uart/data_uart.h"
using namespace std;
void process_data_uart_read(){
    if(data_receive.size()==num_byte_stm_pc){
        // read status live motor
        motor_right_state_live=(int)data_receive[motor1_status_re];
        motor_left_state_live=(int)data_receive[motor2_status_re];
        // read status enable motor
        motor_right_disable=(int)data_receive[motor1_enable_re];
        motor_left_disable=(int)data_receive[motor2_enable_re];
        // read status brake
        motor_right_break=(int)data_receive[motor1_brake_re];
        motor_left_break=(int)data_receive[motor2_brake_re];
        // read status error motor
        motor_right_state_error=(int)data_receive[motor1_error_re];
        motor_left_state_error=(int)data_receive[motor2_error_re];
        // read torque
        tr=(float)data_receive[motor1_torque_L_re]/100;
        tl=(float)data_receive[motor2_torque_L_re]/100;
        // read speed motor right
        static float v_;
        v_=(float)data_receive[motor1_speed_H_re]*100+(float)data_receive[motor1_speed_L_re];
        v_=v_/gear*M_PI*2/60;
        vr=v_;
        if(data_receive[motor1_ccw_re]==1) vr=-vr;
        // read speed motor left
        v_=(float)data_receive[motor2_speed_H_re]*100+(float)data_receive[motor2_speed_L_re];
        v_=v_/gear*M_PI*2/60;
        vl=v_;
        if(data_receive[motor2_ccw_re]==1) vl=-vl;
        // condition for speed read
        if(motor_right_state_live==0 | (motor_right_state_live==1 & motor_right_state_error!=0)) vr=0;
        if(motor_left_state_live==0 | (motor_left_state_live==1 & motor_left_state_error!=0)) vl=0;
        // if speed very small -> speed equal to zero!
        if(fabs(vl)<0.125 & vl_set==0) vl=0;
        if(fabs(vr)<0.125 & vr_set==0) vr=0;
        // read data batterry
        static float check_battery_time_out;
        check_battery_time_out=0;
        for(int i=battery_vol_H_re;i<=battery_vol_cell8_re;i++){
            if(data_receive[i]!=byte_uart_time_out){
                check_battery_time_out=1;
                break;
            }
        }
        if(check_battery_time_out==1){
            battery_live_status=1;
            //
            battery_soc=(float)(data_receive[battery_soc_re]);
            battery_soc=((battery_soc-20)/80)*100;
            if(battery_soc<0) battery_soc=0;
            //
            battery_vol=(float)(data_receive[battery_vol_H_re]*100+data_receive[battery_vol_L_re])/100;
            battery_num_cell=(float)data_receive[battery_num_cell_re];
            battery_cycle=(float)((uint16_t)data_receive[battery_cycle_charing_H_re]<<8 | data_receive[battery_cycle_charing_L_re]);
            battery_guard=(float)((uint16_t)data_receive[battery_guard_H_re]<<8 | data_receive[battery_guard_L_re]);
            battery_mah_now=(float)((uint16_t)data_receive[battery_mah_now_H_re]<<8| data_receive[battery_mah_now_L_re]);
            battery_mah_max=(float)((uint16_t)data_receive[battery_mah_max_H_re]<<8| data_receive[battery_mah_max_L_re]);
            battery_current=(float)((int16_t)(data_receive[battery_current_H_re]<<8 | data_receive[battery_current_L_re]));
            battery_temperature=(float)(data_receive[battery_temperature1_re]+data_receive[battery_temperature1_re])/2;
            //
            for(int i=0;i<8;i++){
                battery_cell[i]=(float)data_receive[battery_vol_cell1_re+i]/10;
            }
        }else battery_live_status=0;
        // read data small batterry
        static float check_battery_small_time_out;
        check_battery_small_time_out=0;
        for(int i=sbattery_vol_H_re;i<=sbattery_temperature1_re;i++){
            if(data_receive[i]!=byte_uart_time_out){
                check_battery_small_time_out=1;
                break;
            }
        }
        if(check_battery_small_time_out==1){
            battery_small_live_status=1;
            //
            battery_small_soc=(float)data_receive[sbattery_soc_re];
            battery_small_temperature=(float)(data_receive[sbattery_temperature0_re]+data_receive[sbattery_temperature1_re])/2;
            battery_small_current=(float)((int16_t)(data_receive[sbattery_current_H_re]<<8 | data_receive[sbattery_current_L_re]));
            battery_small_mah_now=(float)((uint16_t)data_receive[sbattery_mah_now_H_re]<<8| data_receive[sbattery_mah_now_L_re]);
            battery_small_mah_max=(float)((uint16_t)data_receive[sbattery_mah_max_H_re]<<8| data_receive[sbattery_mah_max_L_re]);
            battery_small_num_cell=(float)data_receive[sbattery_num_cell_re];
            battery_small_vol=(float)(data_receive[sbattery_vol_H_re]*100+data_receive[sbattery_vol_L_re])/100;
            
        }else battery_small_live_status=0;
        // input
        for(int i=0;i<num_in_put_user;i++){
            input_user.data[i]=(float)(data_receive[io1_re+i]);
        }
        input_user.data[num_in_put_user+0]=(float)(data_receive[button1_re]);
        input_user.data[num_in_put_user+1]=(float)(data_receive[button2_re]);
        input_user.data[num_in_put_user+2]=(float)(data_receive[button3_re]);
        // encoder
        encoder2=(float)data_receive[robot_hook_encoder_re];
        encoder=((float)data_receive[robot_hook_encoder_re]-encoder_offset)/255*360*encoder_dir;
        if(fabs((fabs(encoder)-360))<fabs(encoder)) encoder=fabs((fabs(encoder)-360))*encoder_dir;
        // hook 
        hook_switch=(float)data_receive[mode_switch_hook];
    }
}
void process_data_uart_write(){
    // motor mode
    if(motor_break==1){
        data_tran[motor1_enable]=(uint8_t)(motor_enable);
        data_tran[motor2_enable]=(uint8_t)(motor_enable);
        data_tran[motor1_break]=1;
        data_tran[motor2_break]=1;
    }else{

        data_tran[motor1_enable]=0;
        data_tran[motor2_enable]=0;
        data_tran[motor1_break]=0;
        data_tran[motor2_break]=0;
    }
    // speed motor left
    if(vl_out>0) data_tran[motor2_cw_cww]=0;
    else data_tran[motor2_cw_cww]=1;
    data_tran[motor2_set_speedH]=(uint8_t)((fabs(vl_out)/(2*M_PI)*60*gear)/100);
    data_tran[motor2_set_speedL]=(uint8_t)((fabs(vl_out)/(2*M_PI)*60*gear)-data_tran[motor2_set_speedH]*100);
    // speed motor right
    if(vr_out>0) data_tran[motor1_cw_cww]=0;
    else data_tran[motor1_cw_cww]=1;
    data_tran[motor1_set_speedH]=(uint8_t)((fabs(vr_out)/(2*M_PI)*60*gear)/100);
    data_tran[motor1_set_speedL]=(uint8_t)((fabs(vr_out)/(2*M_PI)*60*gear)-data_tran[motor1_set_speedH]*100);
    // tourque set
    data_tran[motor1_set_torqueL]=(uint8_t)(torqueR_set*100);
    data_tran[motor2_set_torqueL]=(uint8_t)(torqueL_set*100);
    // color robot
    data_tran[robot_color_red]=(uint8_t)(red);
    data_tran[robot_color_green]=(uint8_t)(green);
    data_tran[robot_color_blue]=(uint8_t)(blue);
    // led robot
    data_tran[robot_led_left]=(uint8_t)(led_l);
    data_tran[robot_led_right]=(uint8_t)(led_r);
    data_tran[robot_led_back]=(uint8_t)(led_b);
    data_tran[robot_led_front]=0;//(uint8_t)(led_f);
    // reset error motor
    if((motor_left_state_error !=0 | motor_right_state_error !=0) & motor_reset==1){
        data_tran[motor1_reset_error]=1;
        data_tran[motor2_reset_error]=1;
        motor_reset=0;
    }else {
        data_tran[motor1_reset_error]=0;
        data_tran[motor2_reset_error]=0;
    }
    // output set
    for(int i=robot_io_user_out0;i<robot_io_user_out0+num_out_put_user;i++){
        data_tran[i]=(uint8_t)output_user.data[i-robot_io_user_out0];
    }
    data_tran[robot_liff_pwm]=(uint8_t)output_user.data[num_out_put_user-1+1];
    data_tran[robot_liff_cylinder]=(uint8_t)output_user.data[num_out_put_user-1+2];
    data_tran[robot_hook_]=(uint8_t)output_user.data[num_out_put_user-1+3];
    data_tran[robot_hook_cylinder]=(uint8_t)output_user.data[num_out_put_user-1+4];
    data_tran[robot_hook_dir]=(uint8_t)output_user.data[num_out_put_user-1+5];
    // robot_shutdown
    data_tran[robot_shutdow]=(uint8_t)robot_shutdown;
    // battery charge
    data_tran[battery_big_charge]=(uint8_t)battery1_charge;
    data_tran[battery_small_charge]=(uint8_t)battery2_charge;
}