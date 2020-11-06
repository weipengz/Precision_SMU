/*
NetworkLib.h - Configures the socket server for the controller board
Copyright (c) 2020 Lightwave Lab, Princeton University
*/

// ensure this library description is only included once
#ifndef NetworkLib_h
#define NetworkLib_h

#include <Bridge.h>
#include <BridgeServer.h>
#include <BridgeClient.h>
#include "Globals.h"


// Network interface description
class Network
{
public:
    void setup(void);
    void test_loop(void);
    BridgeServer server;
private:
    String receive_message(BridgeClient* client);
    void send_message(BridgeClient* client, String message);
};

#endif