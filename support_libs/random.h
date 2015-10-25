

/**
* @file debug.h
* @date 2008/06/17 10:46
* @author Sergio MOntenegro
*
* Copyright 2008 DLR
*
* @brief simple misc functions
*/

#pragma once


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif


/** 32 bit random set, a value -2G .. + 2G **/
int32_t randomTT800();

/** 32 bit random set, a value 0 ...  2G **/
uint32_t  randomTT800Positive();


#ifndef NO_RODOS_NAMESPACE
}
#endif

