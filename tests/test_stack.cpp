#include <cassert>
#include "Stack.h"

int main() {
  Stack<int,10> stack;
  stack.push(123);
  assert(stack.top()==123);
  stack.pop();
  for (int i=0; i<5; ++i) {
    stack.push(i);
  }
  assert(stack.size() == 5);
  for (int i=4; i>=0; --i) {
    assert(stack.top()==i);
    stack.pop();
  }
  assert(stack.empty());
  return 0;
}
