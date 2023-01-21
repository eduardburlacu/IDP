// LineFollower.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "LineFollower.h"


float moving_avg(int N, int len, float v[])
{
    /*
    Use it to get rid of random fluctuations in reading.
    N is the number of points considered to the average.
    len is the length of v
    v[] stores the last recorded values of a variable to be averaged.
    if N < len , only considers the last N readings in v[].
    */
    assert(N <= len);
    float average_total = 0.0;
    for (int index = len- N; index < len; index++) {
        average_total += v[index];
    }
    average_total /= N;
    return average_total;
}

float moving_avg(int len, float v[])
{
    float average_total = 0.0;
    for (int index = 0; index < len; index++) {
        average_total += v[index];
    }
    average_total /= len;
    return average_total;
}



int main() {
    float v[] = { 12.0,1.0,2.3};
    std::cout << moving_avg(3,v);
    return 0;
}