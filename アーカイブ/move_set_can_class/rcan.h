#ifndef rcan_h
#define rcan_h
#include <Arduino.h>
#include <FlexCAN.h>

class rcan {
  public:
    void init();
    void read();
    void write();
    void senddata(int u[4]);
    CAN_message_t msg;
    CAN_message_t rxmsg;
  private:
    int cb = 0;
};

#endif
