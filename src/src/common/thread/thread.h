#include <time.h>
#include <pthread.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
// settime process
// process-core
pthread_mutex_t process_mutex_libary[256];
void * process(char name[],long double timer,long n_mux, void  action())
{
  // var for timer
  long double t_start=0;
  long double t_now=0;
  long double delta=0;
  long long tdelay=0;
  unsigned long long n_process=0;
  long double ts_process1=timer;
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
      delta=t_now-t_start-n_process*ts_process1;
      if(delta>=ts_process1) //check timer
      {
        if(n_process+(unsigned long long)(delta/ts_process1)>n_process) n_process+=(unsigned long long)(delta/ts_process1);
        else 
        {
          n_process=0;
          t_start=(long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9;
        }
	      pthread_mutex_lock(&process_mutex_libary[n_mux]);
            //printf("%s start at system time sec:%ld n_sec:%.4lf\n",name,realtime.tv_sec,realtime.tv_nsec*1e-9); 	// kiem tra process
            action();
        pthread_mutex_unlock(&process_mutex_libary[n_mux]);
      }
      else if(delta>=0.005) {
      		tdelay=(long long)((ts_process1-delta)/2*1e3);
      		if(tdelay>=1000000) sleep((long long)(tdelay/1000000));
      		else usleep((useconds_t)tdelay); 
      	}
    }
    else
    {
      t_start=(long double)realtime.tv_sec+(long double)realtime.tv_nsec*1e-9;
      n_process=0;
    }
  }
}
