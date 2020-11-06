/*
NetworkLib.h - Configures the socket server for the controller board
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

// ensure this library description is only included once
#ifndef NetworkLib_h
#define NetworkLib_h

#include <BridgeServer.h>
#include <BridgeClient.h>
#include "Globals.h"

// Network interface description
class Network
{
public:
    void setup(void);
    void test_loop(void);
private:
    BridgeServer server;
    String receive_message(BridgeClient* client);
    void send_message(BridgeClient* client, String message);
};

#endif
