#include "rodos.h"
#include "demo_topics.h"


class PrintTopics : public Putter {
    bool putGeneric(const long topicId, const unsigned int msgLen, const void* msg, const NetMsgInfo& netMsgInfo) {
            TopicListReport* tr = (TopicListReport*)msg;
            PRINTF("from topic %ld from node %d : numOfTopics %d: ", topicId, netMsgInfo.senderNode, tr->numberOfTopics);
            for(int i = 0; i < tr->numberOfTopics; i++) PRINTF(" %d ", tr->topicList[i]);
            PRINTF("\n");
            return true;
    }
} printTopics;


static Subscriber subsTopicrerpots(defaultGatewayTopic, printTopics, "justprint-Topicreports");

