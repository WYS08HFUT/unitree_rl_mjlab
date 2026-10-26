// deploy/tests/runtime/test_main.cpp
#include <cstdio>

#include "test_framework.h"

void test_keyboard_map();
void test_command_mux();
void test_safety();
void test_behavior();
void test_replay();

int main() {
  test_keyboard_map();
  test_command_mux();
  test_safety();
  test_behavior();
  test_replay();
  std::printf("%d checks, %d failures\n", testfw::checks, testfw::failures);
  return testfw::failures == 0 ? 0 : 1;
}
