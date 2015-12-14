#include "rodos.h"
#include "demo_topics.h"


class JustPrint : public Putter {
    bool putGeneric(const long topicId, const unsigned int msgLen, const void* msg, const NetMsgInfo& netMsgInfo) {
            PRINTF("from topic %ld : %d\n", topicId, *(int32_t*)msg);
            return true;
    }
} justPrint;

static Subscriber subs1(counter1, justPrint, "justprint01");
static Subscriber subs2(counter2, justPrint, "justprint02");
static Subscriber subs3(counter3, justPrint, "justprint03");
static Subscriber subs4(counter4, justPrint, "justprint04");
static Subscriber subs5(counter5, justPrint, "justprint05");



/***************************/

class Onoff : public Thread {
    void run() {
        bool on = true;
        TIME_LOOP(3*SECONDS, 5*SECONDS) {
            on = !on;
            PRINTF("Turing subscriver to %d\n", on);
            subs1.enable(on);
            subs2.enable(on);
            subs3.enable(on);
            subs4.enable(on);
        }
    }

} onOff;

