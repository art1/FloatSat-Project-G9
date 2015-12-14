#include "rodos.h"
#include "demo_topics.h"

/******************************/

class MyPublisher : public Thread {
public:
    MyPublisher() : Thread("sender") { }
    void run () {
        int32_t cnt = 0;
        TIME_LOOP(0, 1000*MILLISECONDS) {
            PRINTF("at %9.3f sending %d\n",  SECONDS_NOW(), cnt);
            counter1.publishConst(cnt+1000);
            counter2.publishConst(cnt+2000);
            counter3.publishConst(cnt+3000);
            counter4.publishConst(cnt+4000);
            counter5.publishConst(cnt+5000);
            cnt++;
        }
    }
} myPublisher;


