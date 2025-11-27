
#include <gtest/gtest.h>

#include <cmath>

using namespace std;

// gtest似乎不支持跨库测试用例，必须将所有的测试用例编译到一个库中
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
