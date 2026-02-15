#include <math.h>

/**
 * 这是一个兼容性补丁，用于连接 Keil 编译的库到 GCC 环境中。
 * Keil 的库引用了以 __ARM_ 开头的数学辅助函数，
 * 我们需要将其重定向到 GCC 提供的标准数学函数上。
 */

float __ARM_scalbnf(float x, int exp) {
    return scalbnf(x, exp);
}

double __ARM_scalbn(double x, int exp) {
    return scalbn(x, exp);
}