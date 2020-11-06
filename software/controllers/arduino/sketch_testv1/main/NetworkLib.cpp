/*
NetworkLib.h - Configures the socket server for the controller board
Copyright (c) 2019 Lightwave Lab, Princeton University
*/

#include "NetworkLib.h"

#include <Bridge.h>
#include <BridgeServer.h>
#include <BridgeClient.h>


void Network::setup()
{
    BridgeServer server(5555);

    // Bridge startup
    Bridge.begin();

    #if SERIAL_DEBUG
        Serial.println("---- Networking Loaded ----");
    #endif

    // initialize server
    server.noListenOnLocalhost();
    server.begin();
}

void Network::test_loop()
{
    /* This loop accepts a connection, then pipes
    any text sent to it to the serial for debug.
    It also receives messages from the Serial port, 
    and sends to client. The messages get buffered
    if connection is dropped.
    */

    #if SERIAL_DEBUG
        BridgeClient client = server.accept();
        if (client) {
            String command;
            String serial_return;
            Serial.println("Client Session Started");
            while(client.connected()) {
                if(client.available()) {
                    Serial.println(client.available());
                    command = receive_message(&client);
                    // command = client.readStringUntil('\n');
                    Serial.println(command);
                }
                if(Serial.available()) {
                    serial_return = Serial.readStringUntil('\n');
                    send_message(&client, serial_return);
                }
            }
            Serial.println("Session Terminated");
            client.stop();
        }
        delay(50);
    #endif //SERIAL_DEBUG
}

/* There is a good reason to use pointers here. When
I used argument by reference the readStringUntil did not
flush the input buffer, almost as if another client
got started. Using pointers allow us to use the same objects.
*/

String Network::receive_message(BridgeClient *client)
/*
 * Receive message from client
 * Returns message as String
 */
{
    String msg;
    msg = client->readStringUntil('\n');
    return msg;
}

void Network::send_message(BridgeClient *client, String message)
/*
 * Send message back to client
 * Input: message - message to send
 */
{
    char buf[SOCKET_BUFFER_SIZE];
    message.concat('\n');
    message.toCharArray(buf, SOCKET_BUFFER_SIZE);
    int written = client->write((uint8_t*) buf, message.length());
}
