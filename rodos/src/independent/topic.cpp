
#include "rodos.h"

/**
 * @file topic.cc
 * @date 2008/09/01 7:07
 * @author Sergio Montenegro, Lutz Dittrich
 *
 * Copyright 2008 DLR
 *
 * @brief topic for middleware
 *
 */

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

/*************************************************/

// List TopicInterface::topicList = 0; This shall be here, but moved to main

static Application applicationName("Topics & Middleware", APID_MIDDLEWARE);


TopicInterface::TopicInterface(long id, long len, const char* name) : ListElement(topicList, name)  {
    mySubscribers = 0;
    topicId = id;
    msgLen = len;
    topicFilter = 0;

    if(topicId == -1) {
        topicId = hash(name) ;
        if(topicId < FIRST_USER_TOPIC_ID) { // reserved topic ids
            topicId +=  FIRST_USER_TOPIC_ID;
        }
    }

    /** Check for replications **/
    ITERATE_LIST(TopicInterface, topicList) {
        if((iter->topicId == id) && (iter != this)) {
            ERROR("Duplicated topicId");
            PRINTF("Duplicated topicId %ld, name1 = %s name2 = %s\n", id, name, iter->name);
        }
    }

}

void TopicInterface::setTopicFilter(TopicFilter* filter) {
    if(topicFilter != 0) {
        ERROR("More than one topicFilter for topic");
        PRINTF(" topic %s has more than one filter\n", name);
    }
    topicFilter = filter;
}

TopicInterface*  TopicInterface::findTopicId(long watedTopicId) {
    ITERATE_LIST(TopicInterface, topicList) {
        if(iter->topicId == watedTopicId)  return iter;
    }
    return 0;
}


/**********************/

unsigned long TopicInterface::publish(void* data, bool shallSendToNetwork, NetMsgInfo* netMsgInfo) {
    return publishMsgPart(data,msgLen,shallSendToNetwork,netMsgInfo);

}

unsigned long TopicInterface::publishMsgPart(void* data, unsigned int lenToSend, bool shallSendToNetwork, NetMsgInfo* netMsgInfo) {
    int cnt = 0; // number of receivers a message is sent to
    NetMsgInfo localmsgInfo;

    if(!netMsgInfo) {
        localmsgInfo.linkId=RODOS_LOCAL_BROADCAST;
        localmsgInfo.sentTime     = NOW();
        localmsgInfo.senderNode   = getNodeNumber();
        localmsgInfo.senderThreadId=(uint32_t)Thread::getCurrentThread();
        netMsgInfo= & localmsgInfo;
    }


    /** If a filter is installed, it may modify the msg bevor the subscriver tet it **/
   if(topicFilter != 0)  topicFilter->modify(topicId, lenToSend, data, *netMsgInfo);

    /** Distribute to all (and only) my subscribers **/
    ITERATE_LIST(Subscriber, mySubscribers) {
        if(iter->isEnabled) cnt += iter->put(topicId, lenToSend, data, *netMsgInfo);
    }

    if(!shallSendToNetwork) { return cnt; }

    /** Now distribute message to all gateways **/
    ITERATE_LIST(Subscriber, defaultGatewayTopic.mySubscribers) {
        cnt += iter->put(topicId, lenToSend, data, *netMsgInfo);
    }
    return cnt;
}

void TopicInterface::publishFromInterrupt(void *any, int len) {
    ITERATE_LIST(Subscriber, mySubscribers) {
        iter->putFromInterrupt(topicId, any, len);
    }
}



// Basic topic used for TBA
Topic<GenericMsgRef> defaultGatewayTopic(0, "gatewayTopic");
Topic<NetworkMessage> defaultRouterTopic(-1, "routerTopic");

Topic<void*> interruptTimer(-1,   "TimerInterrupt");
Topic<void*> interruptUart(-1,    "UartInterrupt");
Topic<void*> interruptSigterm(-1, "SigTermInterrupt");
Topic<GenericMsgRef> charInput(-1, "CharInput");


#ifndef NO_RODOS_NAMESPACE
}
#endif

