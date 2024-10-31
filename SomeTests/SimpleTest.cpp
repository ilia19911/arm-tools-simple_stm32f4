#include <gtest/gtest.h>
#include "arm_math.h"

TEST(SimpleTest, Common) {
  arm_rfft_fast_instance_f32 inst;
  arm_rfft_fast_init_f32(&inst, 64);
  volatile int i = 3;
  printf("start test\n");
  std::cout << i<<std::endl;
}