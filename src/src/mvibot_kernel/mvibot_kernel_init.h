#include "../common/libary/libary_basic.h"
#include "../common/hardware/hardware.h"
#include <CppLinuxSerial/SerialPort.hpp>
#include "../common/data_uart/data_uart.h"
using namespace std;
using namespace mn::CppLinuxSerial;
#if !defined(mvibot_kernel_init)
        //        
        vector<string> list_monitor;
        string system_command;
        // uart
        int uart_enable=0;
        string name_port="/dev/my_USB2";
        SerialPort serialPort(name_port, BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
        vector<uint8_t>  data_tran_uart_defaunt={};
        vector<uint8_t>  data_tran_uart={};
        vector<uint8_t>  data_receive_uart={};
        vector<uint8_t>  data_tran_local={};
        vector<uint8_t>  data_receive_local={};
        vector<uint8_t>  data_tran_socket={};
        vector<uint8_t>  data_tran_socket_defaunt={};
        vector<uint8_t>  data_receive_socket={};
        //
        float time_out_uart;
        float ts_uart;
        float ts_socket;
        float time_out_backup=5;
        // var action
        int send_command_shutdown=0;
        int request_backup=0,robot_backup_reboot=0;
        float button_hold=0,button_hold_backup=0;
        float ts_button_hold;
        // view data
        void view_data(string name, std::vector<uint8_t> data){
            cout<<name;
            for(int i=0;i<data.size();i++){
                printf("0x%x ",data[i]);
            }
            cout<<endl;
        }
    #define mvibot_kernel_init 1
#endif