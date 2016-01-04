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

unsigned long millis() {
  timeval tp;
  gettimeofday(&tp, 0);
  return tp.tv_sec*1000 + tp.tv_usec/1000;
}

void delay(unsigned int ms) {
  cout << "delay of " << ms << " ms" << endl;
  usleep(ms*1000);
}
                
id_type task1_id, task2_id, task3_id;

void task1() {
  cout << "Task1" << endl;
  task1_id=scheduler.timer(TASK1_SLEEP, task1);
}

void task2() {
  cout << "Task2" << endl;
  task2_id=scheduler.timer(TASK2_SLEEP, task2);
}

void task3() {
  cout << "Task3" << endl;
  task3_id=scheduler.timer(TASK3_SLEEP, task3);
}

int main() {
  const int MAX_COUNT=10;
  int count=0;
  task1_id = scheduler.timer(TASK1_SLEEP, task1);
  task2_id = scheduler.timer(TASK2_SLEEP, task2);
  task3_id = scheduler.timer(TASK3_SLEEP, task3);
  while(scheduler.pollWaiting() != ALL_IDLE && count<MAX_COUNT) {
    ++count;
  }
  scheduler.cancel(task1_id);
  scheduler.cancel(task2_id);
  count=0;
  while(scheduler.pollWaiting() != ALL_IDLE && count<MAX_COUNT) {
    ++count;
  }
  return 0;
}
