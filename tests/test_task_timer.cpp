#include <iostream>
#include <cassert>
#include <cstdlib>
extern "C" {
#include <unistd.h>
#include <sys/time.h>
}
#include "TaskTimer.h"

using namespace std;

#define TASK1_SLEEP 1000
#define TASK2_SLEEP 1100
#define TASK3_SLEEP 1500

TaskTimer<10> scheduler;

unsigned long t0=0;
unsigned long t=0;
unsigned long millis() {
  timeval tp;
  gettimeofday(&tp, 0);
  unsigned long cur = (tp.tv_sec*1000 + tp.tv_usec/1000);
  t += cur - t0;
  t0 = cur;
  return t;
}

int count=0;
unsigned long expected_delays[] = {
  1000,100,400,500,200,800,300,700,400,
  100,1500,1500,1500,1500,1500,1500,
  1500,1500,1500,1500,1500,
};
void delay(unsigned long ms) {
  // cout << "delay of " << ms << " ms" << endl;
  if (ms > 4) {
    if ( abs((long)ms - (long)expected_delays[count]) >= 50 ) {
      cout << "delay of " << ms << " ms; "
           << "expected " << expected_delays[count] << " "
           << "at count=" << count << endl;
      assert( abs((long)ms - (long)expected_delays[count]) < 50 );
    }
    count++;
  }
  usleep(ms*1000);
}
                
id_type task1_id, task2_id, task3_id;

void task1() {
  // cout << "Task1" << endl;
  task1_id=scheduler.timer(TASK1_SLEEP, task1);
}

void task2() {
  // cout << "Task2" << endl;
  task2_id=scheduler.timer(TASK2_SLEEP, task2);
}

void task3() {
  // cout << "Task3" << endl;
  task3_id=scheduler.timer(TASK3_SLEEP, task3);
}

int main() {
  for (int i=0; i<2; ++i) {
    count=0;
    const int MAX_COUNT2=10;
    int count2=0;
    task1_id = scheduler.timer(TASK1_SLEEP, task1);
    task2_id = scheduler.timer(TASK2_SLEEP, task2);
    task3_id = scheduler.timer(TASK3_SLEEP, task3);
    assert( scheduler.poll() == WAIT_MORE );
    while(scheduler.pollWaiting() != ALL_IDLE && count2<MAX_COUNT2) {
      ++count2;
    }
    scheduler.cancel(task1_id);
    scheduler.cancel(task2_id);
    count2=0;
    while(scheduler.pollWaiting() != ALL_IDLE && count2<MAX_COUNT2) {
      ++count2;
    }
    scheduler.cancel(task3_id);
    assert( scheduler.poll() == ALL_IDLE );
    
    // next iteration start point
    t = ULONG_MAX - 2000;
    timeval tp;
    gettimeofday(&tp, 0);
    t0 = tp.tv_sec*1000 + tp.tv_usec/1000;
  }
  return 0;
}
