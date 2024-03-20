#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <thread>
using namespace std;
class my_thread{
	public:
		// contructor
		my_thread(string name,long double ts,void function(),bool task_lock,int thread_mutex){
			name_pthread=name;
			ts_set=ts;
			threadPointer=function;
			num_thread=thread_mutex;
			task_mutex=task_lock;
		}
		// start thread
		void start(std::thread  & thread){
			thread = std::thread(&my_thread::loop, this);
		}
		//
		static pthread_mutex_t process_mutex_task;
		static pthread_mutex_t process_mutex_thread[256];
	private:
		string name_pthread="";
		// time set
		long double ts_set=1.0;
		int num_thread=-1;
		// variable for thread
		bool task_mutex=true;
		// loop 
		void loop()
		{
			// var for timer
			long double t_start=0;
			long double t_now=0;
			long double delta=0;
			long long tdelay=0;
			unsigned long long n_process=0;
			long double ts_process=ts_set;
			struct timespec realtime;
			//get time frist run
			clock_gettime(CLOCK_REALTIME, &realtime);
			t_start=(long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9;
			// loop
			while(1)
			{
				// get time now
				clock_gettime(CLOCK_REALTIME, &realtime);
				t_now=(long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9;
				// if time reset then rest var else check timer
				if (t_now-t_start>=0)
				{
					delta=t_now-t_start-n_process*ts_process;
					if(delta>=ts_process) //check timer
					{
						if(n_process+(unsigned long long)(delta/ts_process)>n_process) n_process+=(unsigned long long)(delta/ts_process);
						else 
						{
							n_process=0;
							t_start=(long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9;
						}
						//
						if(task_mutex) pthread_mutex_lock(&process_mutex_task);
							if(num_thread!=-1) pthread_mutex_lock(&process_mutex_thread[num_thread]);
								if(threadPointer!=nullptr) threadPointer();
							if(num_thread!=-1) pthread_mutex_unlock(&process_mutex_thread[num_thread]);
						if(task_mutex) pthread_mutex_unlock(&process_mutex_task);
					}
				else if(delta>=0.005) {
						tdelay=(long long)((ts_process-delta)/2*1e3);
						if(tdelay>=1000000) sleep((long long)(tdelay/1000000));
						else usleep((useconds_t)tdelay); 
					}
				}
				else{
					t_start=(long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9;
					n_process=0;
				}
			}
		}
		//
		void (*threadPointer)() =nullptr;
		
};
pthread_mutex_t my_thread::process_mutex_task=PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
pthread_mutex_t my_thread::process_mutex_thread[256]=PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
void lock(){
	pthread_mutex_lock(&my_thread::process_mutex_task);
}
void unclok(){
	pthread_mutex_unlock(&my_thread::process_mutex_task);
}
//
string get_time(){
	static struct timespec realtime;
	clock_gettime(CLOCK_REALTIME, &realtime);
	string string_return;
	string_return=to_string((long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9);
	return string_return;
}