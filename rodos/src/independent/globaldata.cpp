



/**
* @file globaldata.cc
* @date 2011/04/22 17:08
* @author Sergio Montenegro
*
* @brief Global data which shall be initialize by init
*
*/

#include "rodos.h"


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

/*********************************************/

/**
 * In case a network is avialable, the corresponding node nr.
 */

long myNodeNr = -1 ;
long getNodeNumber() { return myNodeNr; }

/**
 * In case there is a getways: number of arrived messages
 */

int64_t numberOfReceivedMsgsFromNetwork = 0;
int64_t getNumberOfReceivedMsgsFromNetwork() {
	int64_t local_numberOfReceivedMsgsFromNetwork;
	PRIORITY_CEILING{
		local_numberOfReceivedMsgsFromNetwork=numberOfReceivedMsgsFromNetwork;
	}
	return local_numberOfReceivedMsgsFromNetwork;
}

volatile int64_t idleCnt = 0;


#ifndef NO_RODOS_NAMESPACE
}
#endif


