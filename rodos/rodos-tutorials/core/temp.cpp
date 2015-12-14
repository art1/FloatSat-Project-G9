#include "rodos.h"
#include "macros_bits_bytes.h"

class HelloWorld : public Thread {
  void run(){ PRINTF(" %d ones = %b\n", 30, ONES(30)); }
} helloworld;

