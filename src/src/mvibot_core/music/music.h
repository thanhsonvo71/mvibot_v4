#include"../../common/libary/libary_basic.h"
#include"../../common/libary/libary_ros.h"
#include"../../common/hardware/hardware.h"
#include"../mvibot_core_init.h"
using namespace std;
void on_music(int mode){
	if(mode==0) system("killall -9 mplayer");
	else{
		if(status_music_n!=0) system("killall -9 mplayer");
        static string cmd;
        static string file;
        //cmd="mplayer -af channels=2:2:0:0:1:0 -ao alsa:device=hw=1.0 "+define_path+"mp3/";
        cmd="sudo pulseaudio --start && sudo mplayer "+define_path+"mp3/";
 	//
        if(mode==2) file="buzze2.mp3";
		if(mode==1) file="buzze.mp3";
		if(mode==3) file="basic.mp3";
		if(mode==4) file="custom.mp3";
        //
        //basic.mp3 -loop 0 -volume 50 -lirc no &";
        cmd=cmd+file+" -loop 0 -volume "+to_string(volume)+" -lirc no &";
        system(cmd.c_str());
	}
	status_music_n=mode;
}
void off_music(){
	system("killall -9 mplayer");
	status_music_n=0;
}
void music_control(){
    if((motor_right_state_error !=0 & motor_right_state_live ==1) | (motor_left_state_error !=0  & motor_left_state_live ==1)){
		if(status_music_n!=1) on_music(1);
	}else{
		if(status_music_n!=start_music_n) on_music(start_music_n);
	}
}