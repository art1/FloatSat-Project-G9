
#include "rodos.h"
#include "gateway.h"



static UDPInOut udp(-50000);
static LinkinterfaceUDP linkinterfaceUDP(&udp);
static Gateway gateway1(&linkinterfaceUDP, true);

static TopicReporter topicReporter(&gateway1);


#if 0
class GatewayInitiator : public Initiator {
    void init() {
        topicReporter.addGateway(&gateway1);
    }
} gatewayInitiator;
#endif

