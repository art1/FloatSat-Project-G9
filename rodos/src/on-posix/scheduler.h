

/*********************************************************** Copyright 
 **
 ** Copyright (c) 2008, German Aerospace Center (DLR)
 ** All rights reserved.
 ** 
 ** Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are
 ** met:
 ** 
 ** 1 Redistributions of source code must retain the above copyright
 **   notice, this list of conditions and the following disclaimer.
 ** 
 ** 2 Redistributions in binary form must reproduce the above copyright
 **   notice, this list of conditions and the following disclaimer in the
 **   documentation and/or other materials provided with the
 **   distribution.
 ** 
 ** 3 Neither the name of the German Aerospace Center nor the names of
 **   its contributors may be used to endorse or promote products derived
 **   from this software without specific prior written permission.
 ** 
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 ** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 ** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 ** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 ** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 ** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 ** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 ** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 ** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **  
 ****************************************************************************/


/**
* @file scheduler.h
* @date 2010/09/24 16:23
* @author Sergio Montenegro
*
* Copyright 2008 DLR
*
* @brief priority based %scheduler (header)
*
*/

#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__
namespace RODOS {

/**
* @class Scheduler
* @brief A class for the scheduling algorithm.
*
* Priority based scheduler.
*/
class Scheduler {

private:
  static unsigned long long scheduleCounter;

public:
  /**
  * Call the scheduling algorithm.
  */
  static void schedule();

  /**
  * Activate the idle thread.
  */
  static void idle();

  /**
  *  return schedule_counter
  */
  static unsigned long long getScheduleCounter();

  static bool isSchedulerRunning();
};

}
#endif /* __SCHEDULER_H */
