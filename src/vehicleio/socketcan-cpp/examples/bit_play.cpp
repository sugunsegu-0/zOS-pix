#include "socketcan_cpp/socketcan_cpp.h"
#include <string>
#include <iostream>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>
#include <unistd.h>
#include <thread>

using namespace std;

int inputVal = 1;
int bitPosition = 20;
int byteLen = 7;

const int steerMaxAngle = 30;
const int steerMinAngle = -30;
const int steerMaxInput1 = 3;
const int steerMaxInput2 = 234;
const int steerMinInput1 = 0;
const int steerMinInput2 = 0;

int steerMaxVal = (steerMaxInput1*255) + steerMaxInput2;
int steerMinVal = (steerMinInput1*255) + steerMinInput2;


int input_parse(int rec_val,int signal_min,int signal_max,int adc_min,int adc_max)
{
    
    int left_pulse = signal_max - signal_min;
    int right_pulse = adc_max - adc_min;

    // Convert the left range into a 0-1 range (float)

    float valueScaled = (rec_val - signal_min) / (float)left_pulse;

    return adc_min + (valueScaled * right_pulse);
}

int* bit_map(int inputVal,int bitPosition,int byteLen)
{
    int sendPos[64];
    static int steerData[64];

    int steeringAngleConverted = input_parse(inputVal,steerMinAngle, steerMaxAngle,steerMinVal, steerMaxVal);

    // int* ptr = decToBinary(steeringAngleConverted);
    
    int binaryNum[64];
 
    // counter for binary array
    int len = 0;
    while (steeringAngleConverted > 0) {
 
        // storing remainder in binary array
        binaryNum[len] = steeringAngleConverted % 2;
        steeringAngleConverted = steeringAngleConverted / 2;
        len++;
        }

    for(int i=0;i<64;i++){
        sendPos[i]=i;
        steerData[i]=0;

    }
    
    int pos = 63 - bitPosition;
    int startPos1 = sendPos[pos];
    // std::cout<<startPos1<<" ";

    int endPos1;
    int startPos2;
    for (int i=startPos1; i<(startPos1+byteLen);i++)
    {
    if ((i+1) % 8 == 0)
    {
            endPos1 = i;
        if ((endPos1 - startPos1 + 1) == byteLen)
            startPos2 = 0;
        if ((endPos1 - startPos1 + 1) < byteLen)
            startPos2 = i - 15;
        break;
    }
    endPos1 = i;
    }
    // std::cout<<endPos1<<std::endl;

    int bitLen1 = endPos1 - startPos1 + 1;
    int bitLen2 = byteLen - bitLen1;
    int endPos2 = (startPos2 - 1) + bitLen2;

    for(int i=0;i<64;i++){
        sendPos[i]=i;
        steerData[i]=0;
    }
    int j = 0;
    while(1){
    
    if (endPos2 >= startPos2){
        steerData[endPos2] = binaryNum[j];
        // std::cout<<endPos2<<" ";
        j += 1;
        endPos2 -= 1;
    }
    if (endPos2 < startPos2 and j<=len){

        steerData[endPos1] = binaryNum[j];
        j += 1;
        endPos1 -= 1;
    }
    if (j >=len)
        break;
    }

    for(int i=0;i<64;i++){
        if(i%8==0){
            std::cout<<"\n";
        }
        std::cout<<steerData[i]<<" ";
    }

    return steerData;

}


int main()
{
    // input steering angle 30 degree 

    int* ptr = bit_map(30,31,16);
    

    return 0;
}
