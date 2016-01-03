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
#ifndef STACK_H
#define STACK_H

template<typename T, int MAX>
class Stack {
public:
  Stack();
  void push(const T &x);
  T top() const;
  void pop();
  int size() const;
  int capacity() const;
  bool empty() const { return sz==0; }
  
private:
  int sz;
  T stack[MAX];
};

template<typename T, int MAX>
Stack<T,MAX>::Stack() : sz(0) {}

template<typename T, int MAX>
void Stack<T,MAX>::push(const T &x) { stack[sz++] = x; }

template<typename T, int MAX>
T Stack<T,MAX>::top() const { return stack[sz-1]; }

template<typename T, int MAX>
void Stack<T,MAX>::pop() { --sz; }

template<typename T, int MAX>
int Stack<T,MAX>::size() const { return sz; }

template<typename T, int MAX>
int Stack<T,MAX>::capacity() const { return MAX; }

#endif // STACK_H
