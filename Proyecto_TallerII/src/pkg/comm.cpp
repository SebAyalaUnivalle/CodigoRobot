#include "comm.h"

comm::comm(int cmdPin, int statePin, int rxPin, int txPin)
    : bt(cmdPin, statePin, rxPin, txPin) {}

void comm::begin(long baud) {
    bt.begin(baud);
}

bool comm::isConnected() {
    return bt.connected();
}

bool comm::available() {
    return bt.available();
}

String comm::readCommand() {
    String data = "";
    while (bt.available()) {
        char c = bt.read();
        if (c == '\n') break;
        data += c;
    }
    return data;
}

void comm::sendMessage(String msg) {
    bt.println(msg);
}