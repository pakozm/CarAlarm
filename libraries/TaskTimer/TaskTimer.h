/*
 * This file is part of CarAlarm an Arduino project for a car alarm system.
 *
 * Copyright (c) 2016 Francisco Zamora-Martinez (pakozm@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#ifndef TASK_TIMER_H
#define TASK_TIMER_H

#include "Arduino.h"
#include "MinHeap.h"
#include "Stack.h"

typedef unsigned long time_type;
typedef int id_type;
typedef void (*func_type)();

const id_type WAIT_MORE=-1;
const id_type ALL_IDLE=-2;

void sleep(time_type ms);

// Inspired by: http://jeelabs.org/pub/docs/jeelib/classScheduler.html
template<char MAX=10>
class TaskTimer {
public:
  TaskTimer();
  
  // Return next task to run, WAIT_MORE if there are none ready to run, but
  // there are tasks waiting, or ALL_IDLE if there are no tasks waiting
  id_type poll() const;
  
  // Runs next available task, using Sleepy::loseSomeTime() for waiting and
  // returns its identifier.
  id_type pollWaiting();
  
  // set a task timer, in ms
  id_type timer(time_type ms, func_type func);
  
  // cancel a task timer 
  void cancel(id_type id);

private:
  void removeCanceledTasks();
  
  struct task_t {
    time_type when; // in millis value
    func_type func;
    task_t(time_type when=0, func_type func=0) : when(when), func(func) {}
  };

  struct key_func_t {
    const TaskTimer *task_timer;
    key_func_t(const TaskTimer *task_timer) : task_timer(task_timer) {}
    time_type operator()(const id_type &id) const {
      return task_timer->tasks[id].when;
    }
  };
  
  task_t tasks[MAX]; // an array
  MinHeap<id_type,MAX,key_func_t,time_type> tasks_heap;
  Stack<id_type,MAX> ids_stack;
};

template<char MAX>
TaskTimer<MAX>::TaskTimer() :
  tasks_heap(key_func_t(this)) {
  for (int i=0; i<MAX; ++i) {
    ids_stack.push(i);
  }
}

template<char MAX>
id_type TaskTimer<MAX>::poll() const {
  if (!tasks_heap.empty()) {
    id_type id = tasks_heap.top();
    time_type t = millis();
    if (t < tasks[id].when) return WAIT_MORE;
    return id;
  }
  else {
    return ALL_IDLE;
  }
}

template<char MAX>
id_type TaskTimer<MAX>::pollWaiting() {
  removeCanceledTasks();
  if (!tasks_heap.empty()) {
    id_type id = tasks_heap.top();
    task_t task = tasks[id];
    time_type ETA = task.when - millis();
    if (ETA > 0) sleep(ETA);
    task.func();
    tasks_heap.pop();
    ids_stack.push(id);
    return id;
  }
  else {
    return ALL_IDLE;
  }
}

template<char MAX>
id_type TaskTimer<MAX>::timer(time_type ms, func_type func) {
  id_type id = ids_stack.top();
  ids_stack.pop();
  task_t task = tasks[id];
  task.when = millis() + ms;
  task.func = func;
  tasks_heap.push(id);
  return id;
}

// cancel a task timer
template<char MAX>
void TaskTimer<MAX>::cancel(id_type id) {
  tasks[id].func = 0;
}

template<char MAX>
void TaskTimer<MAX>::removeCanceledTasks() {
  while(!tasks_heap.empty() && tasks[tasks_heap.top()].func==0) {
    tasks_heap.pop();
  }
}

#endif // TASK_TIMER_H
