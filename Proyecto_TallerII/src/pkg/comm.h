#ifndef COMM_H
#define COMM_H

#include "../utils/HC05.h"

class comm {

    private: 
    HC05 bt; 

    public:
    //constructor
    comm(int cmdPin, int statePin, int rxPin, int txPin);

    //Metodos principales
    void begin (long baud = 9600);
    bool isConnected();
    bool available();
    String readCommand();
    void sendMessage(String msg);
};
#endif