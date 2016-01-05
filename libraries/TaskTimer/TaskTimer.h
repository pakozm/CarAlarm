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

#ifndef Arduino_h
extern "C"{
#include <limits.h>
}
#endif

#include "Arduino.h"
#include "MinHeap.h"
#include "Stack.h"

typedef unsigned long time_type; ///< This type is for time data.
typedef int id_type;             ///< Type for task identification.
typedef void (*func_type)();     ///< Task function interface.

const id_type WAIT_MORE=-1; ///< Indicates more time for polling a task.
const id_type ALL_IDLE=-2;  ///< Indicates no jobs are pending.

/**
 * @brief This sleep function allow to use JeeLib or delay depending in the
 * sleep time.
 */
void sleep(time_type ms);

/**
 * @brief This class allow to schedule in time simple functions without
 * arguments.
 *
 * This class uses a MinHeap to order all timers by their execution time in
 * mili-seconds (as returned by millis() function). Overflow is avoided by using
 * a secondary stack with such tasks which execution time has overflowed current
 * millis() value. When MinHeap is empty, all pending tasks at the secondary
 * stack are pushed into this MinHeap.
 *
 * @note The maximum number of mili-seconds for any timer is 99999999. Any timer
 * with more than this number of mili-seconds will produce unknown behavior.
 *
 * @note Inspired by: http://jeelabs.org/pub/docs/jeelib/classScheduler.html
 */ 
template<char MAX=10>
class TaskTimer {
public:
  TaskTimer();
  
  /**
   * @brief Return next task to run, WAIT_MORE if there are none ready to run, but
   * there are tasks waiting, or ALL_IDLE if there are no tasks waiting
   */
  id_type poll();
  
  /**
   * @brief Runs next available task, using Sleepy::loseSomeTime() for waiting and
   * returns its identifier.
   */
  id_type pollWaiting();
  
  /// Set a task timer, in ms.
  id_type timer(time_type ms, func_type func);
  
  /// Cancel a task timer.
  void cancel(id_type id);

  /// Executes the given task canceling it.
  void run(id_type id);
  
private:
  /// Traverses heap tasks until next one not cancelled.
  void removeCancelledTasks();

  /// Returns ms until execution of given task.
  time_type computeSleepTime(id_type id) const;

  /// Pushes pending tasks into the heap only if the heap is empty.
  void movePendingToHeap();

  /// A task is defined by a time in millis type and a function.
  struct task_t {
    time_type when; ///< In millis value.
    func_type func; ///< A function without arguments and no return value.
    task_t(time_type when=0, func_type func=0) : when(when), func(func) {}
  };

  /// This key allow to extract the key value from heap objects.
  struct key_func_t {
    const TaskTimer *task_timer;
    key_func_t(const TaskTimer *task_timer) : task_timer(task_timer) {}
    /// Retrusn the when field of the given task id.
    time_type operator()(const id_type &id) const {
      return task_timer->tasks[id].when;
    }
  };

  /// A list of tasks.
  task_t tasks[MAX];
  /// Tasks are ordered by its when field.
  MinHeap<id_type,MAX,key_func_t,time_type> tasks_heap;
  /// A stack of free id values.
  Stack<id_type,MAX> ids_stack;
  /// A secondary stack with pending tasks to avoid millis() overflow problem.
  Stack<id_type,MAX> pending_stack;

  /// Maximum number of mili-seconds for any timer task.
  static const time_type MAX_TIMER_TIME = 99999999; // 99999.999 seconds
};

template<char MAX>
TaskTimer<MAX>::TaskTimer() :
  tasks_heap(key_func_t(this)) {
  for (int i=0; i<MAX; ++i) {
    ids_stack.push(i);
  }
}

template<char MAX>
id_type TaskTimer<MAX>::poll() {
  removeCancelledTasks();
  movePendingToHeap();
  if (!tasks_heap.empty()) {
    id_type id = tasks_heap.top();
    time_type sleep_time = computeSleepTime(id);
    if (sleep_time > 0) return WAIT_MORE;
    return id;
  }
  else {
    return ALL_IDLE;
  }
}

template<char MAX>
id_type TaskTimer<MAX>::pollWaiting() {
  removeCancelledTasks();
  movePendingToHeap();
  if (!tasks_heap.empty()) {
    id_type id = tasks_heap.top();
    time_type sleep_time = computeSleepTime(id);
    sleep(sleep_time);
    run(id);
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
  task_t &task = tasks[id];
  time_type t = millis();
  task.when = t + ms;
  task.func = func;
  // push it at pending_stack when timer overflows
  if (task.when < t) {
    pending_stack.push(id);
  }
  // otherwise push it at tasks_heap
  else {
    tasks_heap.push(id);
  }
  return id;
}

// cancel a task timer
template<char MAX>
void TaskTimer<MAX>::cancel(id_type id) {
  if (tasks_heap.top() == id) {
    ids_stack.push( tasks_heap.top() );
    tasks_heap.pop();
  }
  else {
    tasks[id].func = 0;
  }
}

template<char MAX>
void TaskTimer<MAX>::removeCancelledTasks() {
  while(!tasks_heap.empty() && tasks[tasks_heap.top()].func==0) {
    ids_stack.push( tasks_heap.top() );
    tasks_heap.pop();
  }
}

template<char MAX>
time_type TaskTimer<MAX>::computeSleepTime(id_type id) const {
  const task_t &task = tasks[id];
  time_type t = millis(), sleep_time=0;
  if (t < task.when) {
    time_type dt = task.when - t;
    if (dt < MAX_TIMER_TIME) {
      sleep_time = dt;
    }
  }
  else if (t - task.when > MAX_TIMER_TIME) {
    sleep_time = ULONG_MAX - t + task.when;
  }
  return sleep_time;
}

template<char MAX>
void TaskTimer<MAX>::run(id_type id) {
  const task_t &task = tasks[id];
  task.func();
  cancel(id);
}

template<char MAX>
void TaskTimer<MAX>::movePendingToHeap() {
  if (tasks_heap.empty() && !pending_stack.empty()) {
    while(!pending_stack.empty()) {
      tasks_heap.push( pending_stack.top() );
      pending_stack.pop();
    }
    removeCancelledTasks();
  }
}

#endif // TASK_TIMER_H
