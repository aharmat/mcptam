#include <mcptam/EntropyComputation.h>
#include <gtest/gtest.h>

//#define TEST_EXPRESSION(a) EXPECT_EQ((a), meval::EvaluateMathExpression(#a))


TEST(EntropyComputation, basicOperations){
    EXPECT_FLOAT_EQ(1.418938196, compute_point_entropy_scalar(1));
}

TEST(EntropyComputation, badInput){
    EXPECT_FLOAT_EQ(-1.0, compute_point_entropy_scalar(0));
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
