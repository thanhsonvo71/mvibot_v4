#include <stdio.h>
#include <iostream>
#include "thread_v2.h"
using namespace std;
//
void function1(){
	 cout<<"Thread 1 "<<get_time()<<endl;
}
void function2(){
	 cout<<"Thread 2 "<<get_time()<<endl;
}
int main(){
	//
	pthread_mutex_lock(&my_thread::process_mutex_task);
	std::thread thread1,thread2;
	my_thread my_thread1("Thread 1",0.5,function1,false,0);
	my_thread my_thread2("Thread 2",0.05,function2,false,0);
	my_thread1.start(thread1);
	my_thread2.start(thread2);
	//
	sleep(10);
	pthread_mutex_unlock(&my_thread::process_mutex_task);
	while(1){

	}
	return 0;
}
