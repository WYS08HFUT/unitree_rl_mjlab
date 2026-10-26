// deploy/tests/runtime/test_framework.h
#pragma once

#include <cstdio>

namespace testfw {
inline int checks = 0;
inline int failures = 0;
}  // namespace testfw

#define CHECK(cond)                                                       \
  do {                                                                    \
    ++testfw::checks;                                                     \
    if (!(cond)) {                                                        \
      ++testfw::failures;                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
    }                                                                     \
  } while (0)
