
#include "rodos.h"

#define LOW_STACK_LIMIT 300

/** Checks percent usage of each stack **/

/**************************************************/
class Innocent : public Thread {
public:
    Innocent(const char* name) : Thread(name) { }
    void run() { TIME_LOOP(1*SECONDS, 1*SECONDS) { PRINTF("Innocent %s\n", name); } }
};

Innocent ino1("1");
Innocent ino2("2");
Innocent ino3("3");
Innocent ino4("4");

/*********** Otehre Threads to test stack occupation ***/

char dummyWriter;
void stackUser() {
    static int consumed = 0;
    char variableOnStack[100];
    consumed += 100;
    for(int i = 0; i < 100; i++) variableOnStack[i] = 0x5a;
    dummyWriter = variableOnStack[10];
    PRINTF("Stackconsumer using %d\n", consumed);
    Thread::suspendCallerUntil(NOW() + 1*SECONDS);
    stackUser();
}


/** Consumes more an more stack until it crases *****/

class StackConsumer : public Thread {
public:
    StackConsumer() : Thread("StackConsumer", 400, 2000) { }
    void run() { stackUser(); }
} stackConsumer;

