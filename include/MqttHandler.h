/*
 * MqttHandler.h
 *
 *  Created on: Nov 5, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_MQTTHANDLER_H_
#define INCLUDE_MQTTHANDLER_H_

#include <iostream>
#include <cstdlib>
#include <string>
#include <map>
#include <vector>
#include <cstring>
#include "mqtt/client.h"
#include "mqtt/message.h"
#include "communicator.h"
#include <glog/logging.h>

const std::string MQTT_DFLT_SERVER_ADDRESS { "tcp://localhost:1883" };
const std::string MQTT_DFLT_CLIENT_ID { "NPW_APP" };
const std::string MQTT_DFLT_PERSIST_DIR { "./persist" };
//const char* MQTT_DFLT_LWT_PAYLOAD = "Last will and testament.";

const std::string MQTT_DFLT_TOPIC { "array" };
const int MQTT_DFLT_QOS = 1;
const bool MQTT_DFLT_CLEAN_SESSION = false;
const auto MQTT_DFLT_TIMEOUT = std::chrono::seconds(10);
const std::string DFLT_MQTT_CMD_TOPIC("command_topic");


/////////////////////////////////////////////////////////////////////////////
// Class to receive callbacks

class user_callback: public virtual mqtt::callback {
    bool connected_;
    communicator * commPtr;

    void connection_lost(const std::string& cause) override {
        connected_ = false;
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
            std::cout << "\tcause: " << cause << std::endl;
    }

    void delivery_complete(mqtt::delivery_token_ptr tok) override {
        std::cout << "\n\t[Delivery complete for token: "
                << (tok ? tok->get_message_id() : -1) << "]" << std::endl;
    }
    void connected(const std::string& cause) override;

	void message_arrived(mqtt::const_message_ptr msg) override;

    bool isConnected() {
        return connected_;
    }

public:
    user_callback() {
        connected_ = false;
        commPtr = NULL;
    }
    void setCommunicator(communicator * c);
};

class MqttCommunicator: public communicator {
private:
    std::string address, clientID, publishTopic, persistDir, commandTopic;

    mqtt::message willmsg;
    mqtt::async_client client; //(address, clientID);
    user_callback cb;
    mqtt::connect_options conopts;
    mqtt::will_options willOpts;
    mqtt::token_ptr conntok;
    bool isConnected;
    bool cleanSession;
    unsigned int QoS;
    std::chrono::duration<int64_t> timeout;
public:
    MqttCommunicator(EdgeDevice *);
    void connect() override;
    void disconnect() override;
    void sendMessage(const char * message, const unsigned int length) override;
    bool processIncomingMessage(const char * message, const int length) override;

    virtual void sendQueuedMessagesThread() override;
    ~MqttCommunicator();
    void subscribe() override;
};

#endif /* INCLUDE_MQTTHANDLER_H_ */
