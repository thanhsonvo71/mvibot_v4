#include "../../common/exec/exec.h"
#include "../../common/data_uart/data_uart.h"
#include <CppLinuxSerial/SerialPort.hpp>
using namespace std;
using namespace mn::CppLinuxSerial;
//
extern string name_port;
extern SerialPort serialPort;
extern vector<uint8_t>  data_tran_uart_defaunt;
extern vector<uint8_t>  data_tran_uart;
extern vector<uint8_t>  data_receive_uart;
extern vector<uint8_t>  data_tran_local;
extern vector<uint8_t>  data_receive_local;
extern vector<uint8_t>  data_tran_socket;
extern vector<uint8_t>  data_tran_socket_defaunt;
extern vector<uint8_t>  data_receive_socket;
//
extern float time_out_uart;
extern float ts_uart;
extern int uart_enable;
extern int send_command_shutdown;
extern int request_backup,robot_backup_reboot;
extern float button_hold,button_hold_backup;
extern float ts_button_hold;
extern string define_path,system_command;
extern void view_data(string name, std::vector<uint8_t> data);
//
uint16_t checksum(uint8_t i, std::vector<uint8_t> data, uint8_t length){
     uint16_t Sum = 0;
     for (i; i < length-2; i++)
         Sum = Sum + data[i];
     Sum = ~Sum;
     return Sum;
}
void uart_open(){
    try{
        serialPort.Open();
    }
    catch(const std::exception& e){

    }
}
void uart_write(){
    uart_enable=1;
    //
    data_tran_local=data_tran_uart;
    if(button_hold >6 ) {
        // data_tran_local[robot_shutdow]=1;
        // send_command_shutdown=1;
    }
    //
    if(button_hold_backup >= 12 ) {
        request_backup=1;
    }
    if(request_backup==1){
        data_tran_local[robot_color_red]=0;
        data_tran_local[robot_color_green]=0;
        data_tran_local[robot_color_blue]=100;
        data_tran_local[robot_led_left]=3;
        data_tran_local[robot_led_right]=3;
        data_tran_local[robot_led_back]=3;
    }
    //
    if(data_tran_local.size()>0){
        if(data_tran_local[robot_shutdow]==0x01){
            data_tran_local[robot_shutdow]=1;
            send_command_shutdown=1;
        }
        if(data_tran_local[robot_shutdow]==0x02 | robot_backup_reboot==1){
            data_tran_local[robot_shutdow]=0;
            send_command_shutdown=2;
        }
    }
    // creat 2 byte check sum
    if(data_tran_local.size()==num_byte_pc_stm){
        static uint16_t val;
        val = checksum(3,data_tran_local, data_tran_local.size()-3);
        cout<<"byte check sum"<<endl;
        printf("%x %x \n",(uint8_t)(val & 0xff) + 1,(uint8_t)(val>>8));
        data_tran_local[data_tran_local.size()-5] = ((uint8_t)(val & 0xff) + 1);
        data_tran_local[data_tran_local.size()-4] = (uint8_t)(val>>8);
    }
    //
    try{
            serialPort.WriteBinary(data_tran_local);
    }catch(const std::exception& e){
            std::cerr << e.what() << '\n';
            uart_enable=0;
    }
}
void uart_read(){
    try{
            serialPort.ReadBinary(data_receive_local);
    }catch(const std::exception& e)
    {
            std::cerr << e.what() << '\n';
            uart_enable=0;
    }
    view_data("data_receive_uart: ",data_receive_local);
    if(send_command_shutdown!=0) {
        // cout<<"ShutDown robot\n";
        system_command="echo ";
        system_command=system_command+"robot shutdown at: `date` "+">> "+define_path+"history/startup.log";
        system(system_command.c_str());
        //
        system("systemctl stop ros_startup1.service");
        system("systemctl stop ros_startup.service");       
        //system("pkill roslaunch");
        sleep(2);
        if(send_command_shutdown==1) system("systemctl poweroff -i");
        //system("shutdown -h now");
        else if(send_command_shutdown==2) system("systemctl reboot -i");
        //system("reboot");
        exit(0);
    }
    // check frame data
    static int data_error;
    data_error=0;
    //
    if(data_receive_local.size()<=6){
        data_error=1;
    }else{
        if(data_receive_local[0]!=0xff | data_receive_local[1]!=0xee | data_receive_local[data_receive_local.size()-1]!=0xff | data_receive_local[data_receive_local.size()-2]!=0xee){
            data_error=1;
        }else{
            // check sum here if error -> data_error=1;
            static uint16_t val;
            val = checksum(3,data_receive_local, data_receive_local.size()-3);
            cout<<"byte check sum"<<endl;
            printf("%x %x %x %x \n",data_receive_local[data_receive_local.size()-5],data_receive_local[data_receive_local.size()-4],(uint8_t)(val & 0xff) + 1,(uint8_t)(val>>8));
            if(data_receive_local[data_receive_local.size()-5] != ((uint8_t)(val & 0xff) + 1) ) data_error=1;
            if(data_receive_local[data_receive_local.size()-4] != (uint8_t)(val>>8) ) data_error=1;
        }
        
    }
    if(data_error==0){
        time_out_uart=0;
        data_receive_uart=data_receive_local;
        static uint8_t button;
        button=data_receive_uart[button2_re];
        if(button==0){
            button_hold+=(float)ts_button_hold;
        }else button_hold=0;
        //
        static uint8_t button2;
        button2=data_receive_uart[button3_re];
        if(button2==0){
            button_hold_backup+=(float)ts_button_hold;
        }else button_hold_backup=0;

    }else{
        button_hold=0;
        button_hold_backup=0;
    }
}
void uart_timeout(){
        time_out_uart+=(float)ts_uart;
        if(time_out_uart>=0.250){
            time_out_uart=0.250;
            cout<<"Time out uart\n";
            data_tran_socket=data_tran_socket_defaunt;
            static string check_usbport;
            check_usbport=exec("ls "+name_port);
            if(check_usbport==name_port+"\n" & uart_enable==0){
                uart_open();
            }
        }else{
            data_tran_socket=data_receive_uart;
            //
            if(data_tran_socket.size()==num_byte_stm_pc){
                data_tran_socket[other_action]=0;
                if(button_hold_backup >= 12) data_tran_socket[other_action]=2;
                if(button_hold >=6 )  data_tran_socket[other_action]=1;
            }
          
        }
}
//
