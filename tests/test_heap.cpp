#include <cassert>
#include <cstdlib>
#include "MinHeap.h"

struct keyfn {
  int operator()(const int &a) const {
    return a;
  }
};

int main() {
  MinHeap<int,10,keyfn,int> heap;
  assert(heap.empty());
  assert(heap.size()==0);
  heap.push(123);
  assert(!heap.empty());
  assert(heap.size()==1);
  assert(heap.top()==123);
  heap.pop();
  assert(heap.empty());
  assert(heap.size()==0);
  for (int i=0; i<10; ++i) {
    heap.push(rand());
  }
  assert(!heap.empty());
  assert(heap.size()==10);
  int last = heap.top();
  while(!heap.empty()) {
    int current = heap.top();
    heap.pop();
    assert(last <= current);
    last = current;
  }
  assert(heap.empty());
  assert(heap.size()==0);
  return 0;
}
