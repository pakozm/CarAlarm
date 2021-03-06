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
#ifndef MIN_HEAP_H
#define MIN_HEAP_H

/**
 * @brief a MinHeap implementation templatized with object type T, a maximum
 * number of elements MAX, a function to extract key order KEYFN, and a key type
 * K.
 *
 * This heap uses two arrays. The first one starts at 0 and contains MAX
 * elements. The second one references first one at position -1, so it starts at
 * position 1 instead of 0 and have MAX elements (MAX+1 if you count 0).
 */
template<typename T, int MAX, typename KEYFN, typename K>
class MinHeap {
public:
  /// Needs an instance of the key function.
  MinHeap(const KEYFN &keyfn = KEYFN());
  /// Returns the object with minimum key.
  T top() const;
  /// Removes the minimum key object.
  void pop();
  /// Introduces a new object in the heap.
  void push(const T &id);
  /// Returns the number of elements in the heap.
  int size() const;
  /// Returns the maximum capacity of the heap.
  int capacity() const;
  /// Indicates if the heap is empty.
  bool empty() const { return sz==0; }
  /// Clears the heap.
  void clear() {
    sz = 0;
  }
  
private:
  int sz; ///< Number of elements.
  T __heap__[MAX]; ///< An array with heap data.
  T *heap; ///< With a non valid 0 position, so it starts at 1.
  KEYFN keyfn; ///< The key function given during construction.

  /// Iterative implementation of heapify operation.
  void heapify(int pos);
};

#define left(x) ((x)<<1)
#define right(x) (((x)<<1)+1)
#define parent(x) ((x)>>1)

template<typename T> void swap(T &a, T &b) { T tmp = a; a = b; b = tmp; }

template<typename T, int MAX, typename KEYFN, typename K>
MinHeap<T,MAX,KEYFN,K>::MinHeap(const KEYFN &keyfn) :
  sz(0), heap(__heap__), keyfn(keyfn) {}

template<typename T, int MAX, typename KEYFN, typename K>
T MinHeap<T,MAX,KEYFN,K>::top() const { return heap[1]; }

template<typename T, int MAX, typename KEYFN, typename K>
void MinHeap<T,MAX,KEYFN,K>::pop() {
  heap[1] = heap[sz--];
  heapify(1);
}

template<typename T, int MAX, typename KEYFN, typename K>
void MinHeap<T,MAX,KEYFN,K>::push(const T &obj) {
  int k = ++sz;
  K key_obj = keyfn(obj);
  while(k>1 && key_obj < keyfn(heap[parent(k)])) {
    swap(heap[k],heap[parent(k)]);
    k = parent(k);
  }
  heap[k] = obj;
}

template<typename T, int MAX, typename KEYFN, typename K>
void MinHeap<T,MAX,KEYFN,K>::heapify(int p) {
  T obj = heap[p];
  K key_obj = keyfn(obj);
  while(true) {
    int l = left(p);
    int r = right(p);
    int min = p;
    if (l <= sz && keyfn(heap[l]) < key_obj) min = l;
    if (r <= sz && keyfn(heap[r]) < keyfn(heap[min])) min = r;
    if (min != p) {
      swap(heap[p], heap[min]);
      p = min;
    }
    else break;
  }
}

template<typename T, int MAX, typename KEYFN, typename K>
int MinHeap<T,MAX,KEYFN,K>::size() const { return sz; }

template<typename T, int MAX, typename KEYFN, typename K>
int MinHeap<T,MAX,KEYFN,K>::capacity() const { return MAX; }

#undef left
#undef right
#undef parent

#endif // MIN_HEAP_H
