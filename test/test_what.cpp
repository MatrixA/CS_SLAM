#include <gtest/gtest.h>



TEST(MyslamTest, OKFINE){
    double a=3.11;
    EXPECT_NEAR(a,3.12,0.03);
}


int main(int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}