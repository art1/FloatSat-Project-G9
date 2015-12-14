
/**
* @file bytesey.h
* @author Sergio MOntenegro
*/

#pragma once
#include "stdint.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif


/**  bigendian is the standard net potocoll. 
 * Warning: Will be set in main.
 * never use beore main (eg never in static constructurs)
 **/
extern bool isHostBigEndian; 


// DEPRECATED!!! Only for conpatilibty
//short swapShort(short sw_);
//long swapLong(long lw_);
//long long swapLonglong(long long llw_);

int16_t swap16(int16_t    sw_);
int32_t swap32(int32_t    lw_);
int64_t swap64(int64_t   llw_);
float   swapFloat(float   fw_); 
double  swapDouble(double dw_);


// bigendian / littleendian dependent funcions

short shortConvertHost2Net(short sw_);             ///< DEPRECATED
long  longConvertHost2Net(long sw_);               ///< DEPRECATED
long long longlongConvertHost2Net(long long sw_);  ///< DEPRECATED

int16_t int16ConvertHost2Net(int16_t sw_); 
int32_t int32ConvertHost2Net(int32_t lw_);
int64_t int64ConvertHost2Net(int64_t llw_); 
float   floatConvertHost2Net(float   fw_); 
double  doubleConvertHost2Net(double dw_); 


#ifndef NO_RODOS_NAMESPACE
}
#endif


