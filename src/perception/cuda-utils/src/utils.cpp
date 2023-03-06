#include "utils.hpp"

int iDivUp(const int a, const int b)
{
    return ((a % b) != 0U) ? ((a / b) + 1U) : (a / b);
}