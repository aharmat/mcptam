#include <mcptam/EntropyComputation.h>
#include <gtest/gtest.h>

//#define TEST_EXPRESSION(a) EXPECT_EQ((a), meval::EvaluateMathExpression(#a))


TEST(EntropyComputation, basicOperations){
EXPECT_FLOAT_EQ(1.4189382, compute_point_entropy_scalar(1));
}

/*TEST(MathExpressions, complexOperations){
  TEST_EXPRESSION(((3 + 4) / 2.0) + 10);
  TEST_EXPRESSION(7 * (1 + 2 + 3 - 2 + 3.4) / 12.7);
  TEST_EXPRESSION((1 + 2 + 3) - (8.0 / 10)); 
}*/


TEST(EntropyComputation, badInput){
  EXPECT_FLOAT_EQ(-1, compute_point_entropy_scalar(0));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
