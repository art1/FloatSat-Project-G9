/**
* @file bytesex.h
* @date 2008/11/21 11:46
* @author Sergio Montenegro
*
* Copyright 2008 DLR
*/

//#include "rodos.h"
#include "bytesex.h"


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

// TODO: SWAPCHARS is already in macros.h
/// exchange values of char variables (a) and (b)
#define SWAPCHARS(_a, _b) {char _tempswap = _a;   _a = _b;   _b = _tempswap; }



int16_t swap16(int16_t sw_);
int32_t swap32(int32_t sw_);
int64_t swap64(int64_t sw_);
float   swapFloat(float   sw_); 
double  swapDouble(double  sw_);
int16_t int16ConvertHost2Net(int16_t sw_); 
int32_t int32ConvertHost2Net(int32_t sw_);
int64_t int64ConvertHost2Net(int64_t sw_); 
float   floatConvertHost2Net(float   sw_); 
double  doubleConvertHost2Net(double sw_); 



int16_t swap16(int16_t sw_) {
    int16_t sw = sw_;
    char *c = (char*)&sw;
    SWAPCHARS(c[0], c[1]);
    return sw;
}

int32_t swap32(int32_t lw_) {
    int32_t lw = lw_;
    char *c = (char*)&lw;
    SWAPCHARS(c[0], c[3]);
    SWAPCHARS(c[1], c[2]);
    return lw;
}

float swapFloat(float fw_) {
    float fw = fw_;
    char *c = (char*)&fw;
    SWAPCHARS(c[0], c[3]);
    SWAPCHARS(c[1], c[2]);
    return fw;
}


int64_t swap64(int64_t llw_) {
    int64_t  llw = llw_;
    char *c = (char*)&llw;
    SWAPCHARS(c[0], c[7]);
    SWAPCHARS(c[1], c[6]);
    SWAPCHARS(c[2], c[5]);
    SWAPCHARS(c[3], c[4]);
    return llw;
}

double swapDouble(double dw_) {
    double  dw = dw_;
    char *c = (char*)&dw;
    SWAPCHARS(c[0], c[7]);
    SWAPCHARS(c[1], c[6]);
    SWAPCHARS(c[2], c[5]);
    SWAPCHARS(c[3], c[4]);
    return dw;
}

int16_t int16ConvertHost2Net(int16_t sw) {
    if(isHostBigEndian) return sw;
    return swap16(sw); 
}

int32_t int32ConvertHost2Net(int32_t lw) {
    if(isHostBigEndian) return lw;
    return swap32(lw); 
}

int64_t int64ConvertHost2Net(int64_t llw) {
    if(isHostBigEndian) return llw;
    return swap64(llw); 
}

float floatConvertHost2Net(float fw) {
    if(isHostBigEndian) return fw;
    return swapFloat(fw); 
}

double doubleConvertHost2Net(double dw) {
    if(isHostBigEndian) return dw;
    return swapDouble(dw); 
}


/// DEPRECATED... 
short shortConvertHost2Net(short sw) { if(isHostBigEndian) return sw; return swap16(sw); } 
/// DEPRECATED... 
long  longConvertHost2Net(long lw) { if(isHostBigEndian) return lw; return swap32(lw); }
/// DEPRECATED... 
long long longlongConvertHost2Net(long long llw) { if(isHostBigEndian) return llw; return swap64(llw); } 


#ifndef NO_RODOS_NAMESPACE
}
#endif



