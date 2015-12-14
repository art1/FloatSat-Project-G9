



/**
 * @file main.cc
 * @date 2008/04/22 17:08
 * @author Lutz Dittrich, Sergio Montenegro
 *
 * Copyright 2008 DLR
 *
 * @brief system initialization and main function
 *
 */
#include "hw_specific.h"
#include "rodos.h"
#include "scheduler.h"


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

/** This shall be in topicInterface, but to do not link if we do not need...*/
List TopicInterface::topicList = 0;

bool isHostBigEndian = false; // will  be updated in main

union ByteSexDetector {
    char asBytes[4];
    long asLong;
};

/********************************/

void initSystem() {
    long num;

    ByteSexDetector byteSexDetector;
    byteSexDetector.asLong = 1;
    isHostBigEndian = byteSexDetector.asBytes[3] == 1; // if bigendian, lsb is in byte 3

    hwInitTime();


    /**** Initiators **********/
    xprintf("Calling Initiators and application initiators\n");
    ITERATE_LIST(Initiator, Initiator::initiatorList) {
        iter->init();
    }
    ITERATE_LIST(Application, Application::applicationList) {
        iter->init();
    }


    /**************** Middleware topics & Subscribers ******/

    xprintf("Distribute Subscribers to Topics\n");
    Subscriber* next;
    // we can not use INTERALTE_LIST because iter->getNext is set to 0 in the loop
    for (Subscriber* iter = (Subscriber*)Subscriber::subscriberList; iter!=0; iter = next) {

        next = (Subscriber*)iter->getNext();
        if (iter->isAGateway) {
            iter->append(defaultGatewayTopic.mySubscribers);
        } else {
            iter->append(iter->topicInterface.mySubscribers);
        }
    }

    if (TopicInterface::topicList != 0) {
        xprintf("List of Middleware Topics:\n");
        ITERATE_LIST(TopicInterface, TopicInterface::topicList) {
            xprintf(" %s  Id = %ld len = %ld.   -- Subscribers:\n", iter->getName(), iter->topicId, iter->msgLen);
            for (Subscriber* subs = (Subscriber*)iter->mySubscribers; subs !=0; subs = (Subscriber*)subs->getNext()) {
                xprintf("     %s\n", subs->getName());
            }
        }
    }


    /************* TimeEvents *************/

    xprintf("\nEvent servers:\n");
    num = TimeEvent::initAllElements();
    if (num != 0) xprintf("	%ld TimeEvent managers\n", num);

    Thread::initializeThreads();
}


/***********************************/


#ifndef NO_RODOS_NAMESPACE
}
using namespace RODOS;
#endif

int main (int argc, char** argv);

int     main_argc;
char**  main_argv;
int main (int argc, char** argv) {
    // hwCommandLineParsing(argc,argv);
    main_argc = argc;
    main_argv = argv;

    hwInit();
    xprintf("RODOS %s OS Version %s\nLoaded Applications:\n", RODOS_VERSION_TEXT, OSVERSION);
    Application::printApplications();

    initSystem();

    xprintf("BigEndianity = %d, cpu-Arc = %s, Basis-Os = %s, Cpu-Speed (K-Loops/sec) = %ld yeildtim (ns) %ld\n",
            getIsHostBigEndian(), getHostCpuArch(), getHostBasisOS(), getSpeedKiloLoopsPerSecond(), getYieldTimeOverhead());
    xprintf("Node Number: HEX: %lx Dec: %ld\n",getNodeNumber(),getNodeNumber());
    xprintf("-----------------------------------------------------\n");


    MAIN();

    Timer::setInterval(PARAM_TIMER_INTERVAL);
    Timer::init(); // here begin the timer interrups

    xprintf("--------------- application running ------------\n");
    Scheduler::idle();

    return 0;
}



