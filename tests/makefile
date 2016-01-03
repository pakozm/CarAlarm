all: test

test_stack: test_stack.cpp ../libraries/TaskTimer/Stack.h
	g++ -o test_stack test_stack.cpp -I ../libraries/TaskTimer/

test_heap: test_heap.cpp ../libraries/TaskTimer/MinHeap.h
	g++ -o test_heap test_heap.cpp -I ../libraries/TaskTimer/

test_task_timer: test_task_timer.cpp ../libraries/TaskTimer/TaskTimer.h ../libraries/TaskTimer/Stack.h ../libraries/TaskTimer/MinHeap.h
	g++ -o test_task_timer test_task_timer.cpp -I ../libraries/TaskTimer/ -I ./ ../libraries/TaskTimer/TaskTimer.cpp

test: test_stack test_heap test_task_timer
	./test_stack
	./test_heap
	./test_task_timer
